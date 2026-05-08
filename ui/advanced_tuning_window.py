import math
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Signal, Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit,
    QDoubleSpinBox, QFormLayout, QGroupBox, QComboBox, QSplitter, QGridLayout, QSlider
)

from core.gain_schedule import GainSet, build_bands, pick_gain_by_voltage
from core.gain_schedule_runtime import ScheduleState, debounce_band_choice, interpolate_gain
from core.ladrc import LADRCConfig, LADRCState, ladrc_step
from core.protocol import FRAME_PARAM_SET, encode_frame, encode_param_set_payload


def smoothstep(t: float) -> float:
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


class WhatIfSimulatorWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Interactive What-If Simulator")
        self.resize(900, 520)
        self.hist_y = deque(maxlen=600)
        self._build_ui()

    def _build_ui(self):
        root = QHBoxLayout(self)
        left = QGroupBox("参数拖动")
        grid = QGridLayout(left)

        self.wc_slider = QSlider(Qt.Horizontal)
        self.wc_slider.setRange(10, 1200)
        self.wc_slider.setValue(200)
        self.wo_slider = QSlider(Qt.Horizontal)
        self.wo_slider.setRange(20, 2000)
        self.wo_slider.setValue(600)
        self.b0_slider = QSlider(Qt.Horizontal)
        self.b0_slider.setRange(1, 800)
        self.b0_slider.setValue(100)
        self.noise_slider = QSlider(Qt.Horizontal)
        self.noise_slider.setRange(0, 200)
        self.noise_slider.setValue(20)

        self.wc_val = QLabel(); self.wo_val = QLabel(); self.b0_val = QLabel(); self.noise_val = QLabel()
        grid.addWidget(QLabel("wc"), 0, 0); grid.addWidget(self.wc_slider, 0, 1); grid.addWidget(self.wc_val, 0, 2)
        grid.addWidget(QLabel("wo"), 1, 0); grid.addWidget(self.wo_slider, 1, 1); grid.addWidget(self.wo_val, 1, 2)
        grid.addWidget(QLabel("b0"), 2, 0); grid.addWidget(self.b0_slider, 2, 1); grid.addWidget(self.b0_val, 2, 2)
        grid.addWidget(QLabel("白噪声"), 3, 0); grid.addWidget(self.noise_slider, 3, 1); grid.addWidget(self.noise_val, 3, 2)

        self.plot = pg.PlotWidget(background="#101418")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.addLegend()
        self.cur_step = self.plot.plot(pen=pg.mkPen('#57cc99', width=2), name='ghost_step_response')

        root.addWidget(left, 1)
        root.addWidget(self.plot, 2)

        for slider in [self.wc_slider, self.wo_slider, self.b0_slider, self.noise_slider]:
            slider.valueChanged.connect(self._update_plot)
        self._update_plot()

    def _update_plot(self):
        wc = self.wc_slider.value() / 10.0
        wo = self.wo_slider.value() / 10.0
        b0 = self.b0_slider.value() / 100.0
        noise_amp = self.noise_slider.value() / 100.0
        self.wc_val.setText(f"{wc:.1f}")
        self.wo_val.setText(f"{wo:.1f}")
        self.b0_val.setText(f"{b0:.2f}")
        self.noise_val.setText(f"{noise_amp:.2f}")

        st = LADRCState()
        cfg = LADRCConfig(b0=max(0.05, b0), wc=max(0.1, wc), wo=max(0.1, wo), u_min=-1e4, u_max=1e4)
        y = 0.0
        ys = []
        for k in range(260):
            r = 100.0 if k > 12 else 0.0
            meas = y + np.random.normal(0.0, noise_amp)
            u, st = ladrc_step(r, meas, 0.01, cfg, st)
            y += 0.01 * (-1.6 * y + 0.08 * u)
            ys.append(y)

        self.cur_step.setData(list(range(len(ys))), ys)


class AdvancedTuningWindow(QWidget):
    send_frame_requested = Signal(bytes)

    def __init__(self, config: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Advanced Tuning Lab")
        self.resize(1200, 780)
        self.config = config
        self.bands = build_bands(config.get("gain_schedule", {}))
        self.state = ScheduleState()
        self.ladrc_state = LADRCState()
        self.sim_mode = False

        self.hist_v = deque(maxlen=800)
        self.hist_kp = deque(maxlen=800)
        self.hist_u = deque(maxlen=800)
        self.hist_z2 = deque(maxlen=800)
        self.hist_u0 = deque(maxlen=800)
        self.hist_uc = deque(maxlen=800)
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
        self.heatmap_btn = QPushButton("调参象限图")
        self.whatif_btn = QPushButton("交互式模拟器")
        self.clear_btn = QPushButton("清空曲线")
        for b in [self.apply_btn, self.sim_btn, self.heatmap_btn, self.whatif_btn, self.clear_btn]:
            cly.addWidget(b)
        cly.addStretch(1)

        cfg = QGroupBox("调参选项")
        fly = QFormLayout(cfg)
        self.debounce = QDoubleSpinBox(); self.debounce.setRange(0, 5000); self.debounce.setValue(300)
        self.interp_mode = QComboBox(); self.interp_mode.addItems(["linear", "step", "smoothstep"])
        self.b0 = QDoubleSpinBox(); self.b0.setRange(0.001, 1000); self.b0.setValue(1.0)
        self.wc = QDoubleSpinBox(); self.wc.setRange(0.1, 1000); self.wc.setValue(20.0)
        self.wo = QDoubleSpinBox(); self.wo.setRange(0.1, 2000); self.wo.setValue(60.0)
        fly.addRow("分段防抖(ms)", self.debounce); fly.addRow("插值模式", self.interp_mode)
        fly.addRow("LADRC b0", self.b0); fly.addRow("LADRC wc", self.wc); fly.addRow("LADRC wo", self.wo)

        split = QSplitter()
        left = QWidget(); lly = QVBoxLayout(left)
        self.plot = pg.PlotWidget(background="#101418")
        self.plot.addLegend(); self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.cur_v = self.plot.plot(pen='y', name='battery_v')
        self.cur_kp = self.plot.plot(pen='c', name='scheduled_kp')
        self.cur_u = self.plot.plot(pen='m', name='ladrc_u')
        self.cur_z2 = self.plot.plot(pen=pg.mkPen('#ff6b6b', width=2), name='eso_z2')
        self.cur_u0 = self.plot.plot(pen=pg.mkPen('#4dabf7', width=1), name='u0(PD)')
        self.cur_uc = self.plot.plot(pen=pg.mkPen('#fa5252', width=1), name='-z2/b0')
        lly.addWidget(self.plot)

        right = QWidget(); rly = QVBoxLayout(right)
        self.log = QTextEdit(); self.log.setReadOnly(True)
        rly.addWidget(cfg); rly.addWidget(QLabel("运行日志")); rly.addWidget(self.log)

        split.addWidget(left); split.addWidget(right); split.setSizes([800, 360])
        root.addWidget(ctrl); root.addWidget(split)

        self.sim_timer = QTimer(self); self.sim_timer.timeout.connect(self._tick_simulation)

    def _bind_events(self):
        self.apply_btn.clicked.connect(self._apply_current)
        self.sim_btn.clicked.connect(self._toggle_simulation)
        self.heatmap_btn.clicked.connect(self._run_tuning_map)
        self.whatif_btn.clicked.connect(self._open_whatif_sim)
        self.clear_btn.clicked.connect(self._clear_curves)

    def _mode_guard(self):
        self.heatmap_btn.setEnabled(not self.sim_mode)

    def _clear_curves(self):
        for h in [self.hist_v, self.hist_kp, self.hist_u, self.hist_z2, self.hist_u0, self.hist_uc]:
            h.clear()
        for c in [self.cur_v, self.cur_kp, self.cur_u, self.cur_z2, self.cur_u0, self.cur_uc]:
            c.setData([])

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
                    return b.name, GainSet(kp=b.gains.kp + (n.gains.kp - b.gains.kp) * t, ki=b.gains.ki + (n.gains.ki - b.gains.ki) * t, kd=b.gains.kd + (n.gains.kd - b.gains.kd) * t)
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
        target = float(tel.get('target', 0.0)); feedback = float(tel.get('feedback', 0.0))
        u, self.ladrc_state = ladrc_step(target, feedback, 0.01, cfg, self.ladrc_state)
        e = target - feedback
        u0 = cfg.wc * cfg.wc * e - 2.0 * cfg.wc * self.ladrc_state.z2
        u_comp = -self.ladrc_state.z2 / max(cfg.b0, 1e-6)

        self.hist_v.append(voltage); self.hist_kp.append(gains.kp); self.hist_u.append(u)
        self.hist_z2.append(self.ladrc_state.z2); self.hist_u0.append(u0); self.hist_uc.append(u_comp)
        x = list(range(len(self.hist_v)))
        self.cur_v.setData(x, list(self.hist_v)); self.cur_kp.setData(x, list(self.hist_kp)); self.cur_u.setData(x, list(self.hist_u))
        self.cur_z2.setData(x, list(self.hist_z2)); self.cur_u0.setData(x, list(self.hist_u0)); self.cur_uc.setData(x, list(self.hist_uc))
        self.log.setText(
            f"mode={self.interp_mode.currentText()} sim={self.sim_mode}\n"
            f"raw={raw_band} active={active} interp={band}\n"
            f"V={voltage:.2f} kp={gains.kp:.3f} ki={gains.ki:.3f} kd={gains.kd:.3f}\n"
            f"LADRC_u={u:.3f} z2={self.ladrc_state.z2:.3f} u0={u0:.3f} comp={u_comp:.3f}"
        )
        self._last_device = device_id; self._last_channel = channel_id

    def _apply_current(self):
        if not self.last_pick:
            self.log.append("[warn] 无可下发参数")
            return
        _, gains = self.last_pick
        payload = encode_param_set_payload(kp=gains.kp, ki=gains.ki, kd=gains.kd)
        frame = encode_frame(FRAME_PARAM_SET, getattr(self, '_last_device', 1), getattr(self, '_last_channel', 0), payload)
        self.send_frame_requested.emit(frame)
        self.log.append("[ok] PARAM_SET 已发送")

    def _toggle_simulation(self):
        self.sim_mode = not self.sim_mode
        if self.sim_mode:
            self.sim_btn.setText("停止模拟"); self.sim_timer.start(50)
        else:
            self.sim_btn.setText("启动模拟"); self.sim_timer.stop()
        self._mode_guard()

    def _tick_simulation(self):
        self.sim_t += 0.05
        voltage = 24.0 - 4.0 * (self.sim_t / 30.0) + 0.15 * math.sin(self.sim_t * 2.3)
        target = 100.0 if self.sim_t > 1.0 else 0.0
        disturbance = -20.0 if 8.0 < self.sim_t < 10.0 else 0.0
        feedback = target * (1 - math.exp(-0.25 * self.sim_t)) + 3.0 * math.sin(self.sim_t * 1.2) + disturbance
        self.update_from_telemetry({"timestamp_ms": int(self.sim_t * 1000), "target": target, "feedback": feedback, "extra1": voltage}, 99, 0)

    def _open_whatif_sim(self):
        self._whatif_win = WhatIfSimulatorWindow(self)
        self._whatif_win.show()

    def _run_tuning_map(self):
        if self.sim_mode:
            self.log.append("[warn] 请先停止模拟，再生成调参地图")
            return
        wcs = np.linspace(8, 44, 31)
        wos = np.linspace(20, 140, 31)
        img = np.zeros((len(wos), len(wcs)))

        for i, wo in enumerate(wos):
            for j, wc in enumerate(wcs):
                st = LADRCState()
                cfg = LADRCConfig(b0=max(0.2, self.b0.value()), wc=float(wc), wo=float(wo), u_min=-1e4, u_max=1e4)
                y = 0.0
                overshoot = 0.0
                noise_gain = 0.0
                for k in range(220):
                    r = 100.0 if k > 10 else 0.0
                    meas = y + np.random.normal(0.0, 0.2)
                    u, st = ladrc_step(r, meas, 0.01, cfg, st)
                    y += 0.01 * (-1.8 * y + 0.09 * u)
                    overshoot = max(overshoot, y - r)
                    noise_gain += abs(st.z2)
                if wc < 14 and wo < 45:
                    cls = 0
                elif overshoot > 25:
                    cls = 1
                elif noise_gain > 2200 or wo / max(wc, 1e-6) > 5.0:
                    cls = 2
                else:
                    cls = 3
                img[i, j] = cls

        cmap = pg.ColorMap([0.0, 0.33, 0.66, 1.0],
                           [(255, 215, 0, 180), (230, 57, 70, 180), (114, 9, 183, 180), (46, 196, 182, 180)])
        w = QWidget(); w.setWindowTitle("LADRC 调参雷达/象限图")
        lay = QVBoxLayout(w); pw = pg.PlotWidget(background="#101418")
        im = pg.ImageItem(img); im.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256)); pw.addItem(im)
        marker_x = (self.wc.value() - wcs[0]) / (wcs[-1] - wcs[0]) * (len(wcs) - 1)
        marker_y = (self.wo.value() - wos[0]) / (wos[-1] - wos[0]) * (len(wos) - 1)
        pw.addItem(pg.InfiniteLine(pos=marker_x, angle=90, pen=pg.mkPen('w', width=1)))
        pw.addItem(pg.InfiniteLine(pos=marker_y, angle=0, pen=pg.mkPen('w', width=1)))
        lay.addWidget(QLabel("黄=太肉了  红=超调区  紫=神经质  绿=甜点区（白色十字=当前参数）")); lay.addWidget(pw)
        w.resize(700, 520); w.show(); self._heatmap_win = w
