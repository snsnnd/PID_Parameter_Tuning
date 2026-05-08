import math
import struct
from collections import deque

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QDoubleSpinBox, QFormLayout, QGroupBox, QComboBox
import pyqtgraph as pg
import numpy as np

from core.gain_schedule import build_bands, pick_gain_by_voltage
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
        self.resize(1000, 700)
        self.config = config
        self.sender = sender
        self.bands = build_bands(config.get("gain_schedule", {}))
        self.state = ScheduleState()
        self.ladrc_state = LADRCState()
        self.hist_v = deque(maxlen=500)
        self.hist_kp = deque(maxlen=500)
        self.hist_ladrc_u = deque(maxlen=500)
        self.sim_t = 0.0

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        self.apply_btn = QPushButton("一键写参(当前分段)")
        self.log = QTextEdit(); self.log.setReadOnly(True)
        top.addWidget(self.apply_btn)
        top.addWidget(QLabel("防抖ms"))
        self.debounce = QDoubleSpinBox(); self.debounce.setRange(0, 5000); self.debounce.setValue(300)
        top.addWidget(self.debounce)
        top.addWidget(QLabel("插值"))
        self.interp_mode = QComboBox(); self.interp_mode.addItems(["linear", "step", "smoothstep"])
        top.addWidget(self.interp_mode)
        self.sim_btn = QPushButton("启动模拟")
        self.scan_btn = QPushButton("LADRC热力图")
        top.addWidget(self.sim_btn)
        top.addWidget(self.scan_btn)
        root.addLayout(top)

        fg = QGroupBox("LADRC 观测/控制建议")
        ff = QFormLayout(fg)
        self.b0 = QDoubleSpinBox(); self.b0.setRange(0.001, 1000); self.b0.setValue(1.0)
        self.wc = QDoubleSpinBox(); self.wc.setRange(0.1, 1000); self.wc.setValue(20.0)
        self.wo = QDoubleSpinBox(); self.wo.setRange(0.1, 2000); self.wo.setValue(60.0)
        ff.addRow("b0", self.b0); ff.addRow("wc", self.wc); ff.addRow("wo", self.wo)
        root.addWidget(fg)

        self.plot = pg.PlotWidget(background="#101418")
        self.cur_v = self.plot.plot(pen='y', name='battery_v')
        self.cur_kp = self.plot.plot(pen='c', name='scheduled_kp')
        self.cur_u = self.plot.plot(pen='m', name='ladrc_u')
        root.addWidget(self.plot)
        root.addWidget(self.log)

        self.apply_btn.clicked.connect(self._apply_current)
        self.sim_btn.clicked.connect(self._toggle_simulation)
        self.scan_btn.clicked.connect(self._run_ladrc_heatmap)
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self._tick_simulation)
        self.last_pick = None

    def update_from_telemetry(self, tel: dict, device_id: int, channel_id: int):
        voltage = float(tel.get("extra1", 0.0))
        raw_pick = pick_gain_by_voltage(voltage, self.bands)
        interp_pick = interpolate_gain(voltage, self.bands)
        if interp_pick and self.interp_mode.currentText() == "step":
            interp_pick = raw_pick
        elif interp_pick and self.interp_mode.currentText() == "smoothstep":
            # local smooth interpolation within current band
            for i, b in enumerate(self.bands):
                if b.min_v <= voltage < b.max_v and i + 1 < len(self.bands):
                    n = self.bands[i+1]
                    t = smoothstep((voltage - b.min_v) / max(1e-6, b.max_v - b.min_v))
                    from core.gain_schedule import GainSet
                    interp_pick = (b.name, GainSet(
                        kp=b.gains.kp + (n.gains.kp - b.gains.kp) * t,
                        ki=b.gains.ki + (n.gains.ki - b.gains.ki) * t,
                        kd=b.gains.kd + (n.gains.kd - b.gains.kd) * t,
                    ))
                    break
        now_ms = int(tel.get("timestamp_ms", 0))
        raw_band = raw_pick[0] if raw_pick else None
        active = debounce_band_choice(raw_band, now_ms, self.state, int(self.debounce.value()))
        self.last_pick = interp_pick if interp_pick else raw_pick
        if self.last_pick:
            band, gains = self.last_pick
            self.hist_v.append(voltage)
            self.hist_kp.append(gains.kp)
            self.cur_v.setData(list(range(len(self.hist_v))), list(self.hist_v))
            self.cur_kp.setData(list(range(len(self.hist_kp))), list(self.hist_kp))
            dt = 0.01
            cfg = LADRCConfig(b0=self.b0.value(), wc=self.wc.value(), wo=self.wo.value(), u_min=-1e4, u_max=1e4)
            u, self.ladrc_state = ladrc_step(tel.get('target', 0.0), tel.get('feedback', 0.0), dt, cfg, self.ladrc_state)
            self.hist_ladrc_u.append(u)
            self.cur_u.setData(list(range(len(self.hist_ladrc_u))), list(self.hist_ladrc_u))
            self.log.setText(f"V={voltage:.2f} raw={raw_band} active={active}\ninterp:{band} kp={gains.kp:.3f} ki={gains.ki:.3f} kd={gains.kd:.3f}\nLADRC_u={u:.3f} mode={self.interp_mode.currentText()}")
            self._last_device = device_id
            self._last_channel = channel_id

    def _apply_current(self):
        if not self.last_pick:
            return
        _, gains = self.last_pick
        payload = struct.pack('<12f4B', gains.kp, gains.ki, gains.kd, -10000.0, 10000.0, -10000.0, 10000.0, 0.0, 0.15, 0.0, 0.0, 0.0, 1, 1, 1, 0)
        frame = encode_frame(FRAME_PARAM_SET, getattr(self, '_last_device', 1), getattr(self, '_last_channel', 0), payload)
        self.sender(frame)

    def _toggle_simulation(self):
        if self.sim_timer.isActive():
            self.sim_timer.stop()
            self.sim_btn.setText("启动模拟")
        else:
            self.sim_timer.start(50)
            self.sim_btn.setText("停止模拟")

    def _tick_simulation(self):
        self.sim_t += 0.05
        voltage = 24.0 - 4.0 * (self.sim_t / 30.0) + 0.15 * math.sin(self.sim_t * 2.3)
        target = 100.0 if self.sim_t > 1.0 else 0.0
        feedback = target * (1 - math.exp(-0.25 * self.sim_t)) + 3.0 * math.sin(self.sim_t * 1.2)
        tel = {
            "timestamp_ms": int(self.sim_t * 1000),
            "target": target,
            "feedback": feedback,
            "extra1": voltage,
        }
        self.update_from_telemetry(tel, 99, 0)

    def _run_ladrc_heatmap(self):
        wcs = np.linspace(8, 40, 25)
        wos = np.linspace(20, 120, 25)
        img = np.zeros((len(wos), len(wcs)))
        for i, wo in enumerate(wos):
            for j, wc in enumerate(wcs):
                st = LADRCState()
                cfg = LADRCConfig(b0=max(0.2, self.b0.value()), wc=float(wc), wo=float(wo), u_min=-1e4, u_max=1e4)
                y = 0.0
                iae = 0.0
                for k in range(200):
                    r = 100.0 if k > 10 else 0.0
                    u, st = ladrc_step(r, y, 0.01, cfg, st)
                    y += 0.01 * (-1.8 * y + 0.09 * u)
                    iae += abs(r - y) * 0.01
                img[i, j] = iae
        w = QWidget(self)
        w.setWindowTitle("LADRC 参数热力图(IAE)")
        lay = QVBoxLayout(w)
        pw = pg.PlotWidget(background="#101418")
        im = pg.ImageItem(img)
        pw.addItem(im)
        lay.addWidget(pw)
        w.resize(640, 480)
        w.show()
        self._heatmap_win = w
