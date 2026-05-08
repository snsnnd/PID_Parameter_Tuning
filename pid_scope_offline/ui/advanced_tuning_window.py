import math
import struct
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit,
    QDoubleSpinBox, QFormLayout, QGroupBox, QComboBox, QSplitter
)

from core.gain_schedule import GainSet, build_bands, pick_gain_by_voltage
from core.gain_schedule_runtime import ScheduleState, debounce_band_choice, interpolate_gain
from core.ladrc import LADRCConfig, LADRCState, ladrc_step
from core.protocol import FRAME_PARAM_SET, encode_frame


def smoothstep(t: float) -> float:
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


class AdvancedTuningWindow(QWidget):
    def __init__(self, config: dict, sender, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Advanced Tuning Lab")
        self.resize(1100, 760)
        self.config = config
        self.sender = sender
        self.bands = build_bands(config.get("gain_schedule", {}))
        self.state = ScheduleState()
        self.ladrc_state = LADRCState()
        self.sim_mode = False

        self.hist_v = deque(maxlen=800)
        self.hist_kp = deque(maxlen=800)
        self.hist_u = deque(maxlen=800)
        self.sim_t = 0.0
        self.last_pick: tuple[str, GainSet] | None = None

        self._build_ui()
        self._bind_events()

    def _build_ui(self):
        root = QVBoxLayout(self)

        ctrl = QGroupBox("控制面板")
        cly = QHBoxLayout(ctrl)
        self.apply_btn = QPushButton("一键写参")
        self.sim_btn = QPushButton("启动模拟")
        self.heatmap_btn = QPushButton("LADRC 热力图")
        self.clear_btn = QPushButton("清空曲线")
        cly.addWidget(self.apply_btn)
        cly.addWidget(self.sim_btn)
        cly.addWidget(self.heatmap_btn)
        cly.addWidget(self.clear_btn)
        cly.addStretch(1)

        cfg = QGroupBox("调参选项")
        fly = QFormLayout(cfg)
        self.debounce = QDoubleSpinBox(); self.debounce.setRange(0, 5000); self.debounce.setValue(300)
        self.interp_mode = QComboBox(); self.interp_mode.addItems(["linear", "step", "smoothstep"])
        self.b0 = QDoubleSpinBox(); self.b0.setRange(0.001, 1000); self.b0.setValue(1.0)
        self.wc = QDoubleSpinBox(); self.wc.setRange(0.1, 1000); self.wc.setValue(20.0)
        self.wo = QDoubleSpinBox(); self.wo.setRange(0.1, 2000); self.wo.setValue(60.0)
        fly.addRow("分段防抖(ms)", self.debounce)
        fly.addRow("插值模式", self.interp_mode)
        fly.addRow("LADRC b0", self.b0)
        fly.addRow("LADRC wc", self.wc)
        fly.addRow("LADRC wo", self.wo)

        split = QSplitter()
        left = QWidget(); lly = QVBoxLayout(left)
        self.plot = pg.PlotWidget(background="#101418")
        self.plot.addLegend()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.cur_v = self.plot.plot(pen='y', name='battery_v')
        self.cur_kp = self.plot.plot(pen='c', name='scheduled_kp')
        self.cur_u = self.plot.plot(pen='m', name='ladrc_u')
        lly.addWidget(self.plot)

        right = QWidget(); rly = QVBoxLayout(right)
        self.log = QTextEdit(); self.log.setReadOnly(True)
        rly.addWidget(cfg)
        rly.addWidget(QLabel("运行日志"))
        rly.addWidget(self.log)

        split.addWidget(left)
        split.addWidget(right)
        split.setSizes([760, 340])

        root.addWidget(ctrl)
        root.addWidget(split)

        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self._tick_simulation)

    def _bind_events(self):
        self.apply_btn.clicked.connect(self._apply_current)
        self.sim_btn.clicked.connect(self._toggle_simulation)
        self.heatmap_btn.clicked.connect(self._run_ladrc_heatmap)
        self.clear_btn.clicked.connect(self._clear_curves)

    def _mode_guard(self):
        self.heatmap_btn.setEnabled(not self.sim_mode)

    def _clear_curves(self):
        self.hist_v.clear(); self.hist_kp.clear(); self.hist_u.clear()
        self.cur_v.setData([]); self.cur_kp.setData([]); self.cur_u.setData([])

    def _interp_pick(self, voltage: float, raw_pick):
        interp = interpolate_gain(voltage, self.bands)
        mode = self.interp_mode.currentText()
        if mode == 'step':
            return raw_pick
        if mode == 'smoothstep' and interp:
            for i, b in enumerate(self.bands):
                if b.min_v <= voltage < b.max_v and i + 1 < len(self.bands):
                    n = self.bands[i + 1]
                    t = smoothstep((voltage - b.min_v) / max(1e-6, b.max_v - b.min_v))
                    return b.name, GainSet(
                        kp=b.gains.kp + (n.gains.kp - b.gains.kp) * t,
                        ki=b.gains.ki + (n.gains.ki - b.gains.ki) * t,
                        kd=b.gains.kd + (n.gains.kd - b.gains.kd) * t,
                    )
        return interp if interp else raw_pick

    def update_from_telemetry(self, tel: dict, device_id: int, channel_id: int):
        voltage = float(tel.get('extra1', 0.0))
        raw_pick = pick_gain_by_voltage(voltage, self.bands)
        now_ms = int(tel.get('timestamp_ms', 0))
        raw_band = raw_pick[0] if raw_pick else None
        active = debounce_band_choice(raw_band, now_ms, self.state, int(self.debounce.value()))
        self.last_pick = self._interp_pick(voltage, raw_pick)
        if not self.last_pick:
            return

        band, gains = self.last_pick
        cfg = LADRCConfig(b0=self.b0.value(), wc=self.wc.value(), wo=self.wo.value(), u_min=-1e4, u_max=1e4)
        u, self.ladrc_state = ladrc_step(float(tel.get('target', 0.0)), float(tel.get('feedback', 0.0)), 0.01, cfg, self.ladrc_state)

        self.hist_v.append(voltage); self.hist_kp.append(gains.kp); self.hist_u.append(u)
        x = list(range(len(self.hist_v)))
        self.cur_v.setData(x, list(self.hist_v))
        self.cur_kp.setData(x, list(self.hist_kp))
        self.cur_u.setData(x, list(self.hist_u))
        self.log.setText(
            f"mode={self.interp_mode.currentText()} sim={self.sim_mode}\n"
            f"raw={raw_band} active={active} interp={band}\n"
            f"V={voltage:.2f} kp={gains.kp:.3f} ki={gains.ki:.3f} kd={gains.kd:.3f}\n"
            f"LADRC_u={u:.3f}"
        )
        self._last_device = device_id
        self._last_channel = channel_id

    def _apply_current(self):
        if not self.last_pick:
            self.log.append("[warn] 无可下发参数")
            return
        _, gains = self.last_pick
        payload = struct.pack('<12f4B', gains.kp, gains.ki, gains.kd, -10000.0, 10000.0, -10000.0, 10000.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1, 1, 1, 0)
        frame = encode_frame(FRAME_PARAM_SET, getattr(self, '_last_device', 1), getattr(self, '_last_channel', 0), payload)
        self.sender(frame)
        self.log.append("[ok] PARAM_SET 已发送")

    def _toggle_simulation(self):
        self.sim_mode = not self.sim_mode
        if self.sim_mode:
            self.sim_btn.setText("停止模拟")
            self.sim_timer.start(50)
        else:
            self.sim_btn.setText("启动模拟")
            self.sim_timer.stop()
        self._mode_guard()

    def _tick_simulation(self):
        self.sim_t += 0.05
        voltage = 24.0 - 4.0 * (self.sim_t / 30.0) + 0.15 * math.sin(self.sim_t * 2.3)
        target = 100.0 if self.sim_t > 1.0 else 0.0
        feedback = target * (1 - math.exp(-0.25 * self.sim_t)) + 3.0 * math.sin(self.sim_t * 1.2)
        self.update_from_telemetry({"timestamp_ms": int(self.sim_t * 1000), "target": target, "feedback": feedback, "extra1": voltage}, 99, 0)

    def _run_ladrc_heatmap(self):
        if self.sim_mode:
            self.log.append("[warn] 请先停止模拟，再生成热力图")
            return
        wcs = np.linspace(8, 40, 25)
        wos = np.linspace(20, 120, 25)
        img = np.zeros((len(wos), len(wcs)))
        for i, wo in enumerate(wos):
            for j, wc in enumerate(wcs):
                st = LADRCState()
                cfg = LADRCConfig(b0=max(0.2, self.b0.value()), wc=float(wc), wo=float(wo), u_min=-1e4, u_max=1e4)
                y = 0.0; iae = 0.0
                for k in range(200):
                    r = 100.0 if k > 10 else 0.0
                    u, st = ladrc_step(r, y, 0.01, cfg, st)
                    y += 0.01 * (-1.8 * y + 0.09 * u)
                    iae += abs(r - y) * 0.01
                img[i, j] = iae

        w = QWidget(self); w.setWindowTitle("LADRC 参数热力图(IAE)")
        lay = QVBoxLayout(w); pw = pg.PlotWidget(background="#101418")
        im = pg.ImageItem(img); pw.addItem(im)
        lay.addWidget(QLabel("横轴: wc, 纵轴: wo, 颜色: IAE")); lay.addWidget(pw)
        w.resize(640, 500); w.show(); self._heatmap_win = w
