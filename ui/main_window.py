import json
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QComboBox, QListWidget, QFormLayout, QDoubleSpinBox, QGroupBox, QTextEdit
)
import pyqtgraph as pg

from transport.serial_worker import SerialWorker
from core.frame import PIDFrame
from core.protocol import FRAME_TELEMETRY, decode_telemetry
from core.device_manager import DeviceManager
from core.analyzer import basic_step_analysis
from core.gain_schedule import build_bands, pick_gain_by_voltage
from ui.advanced_tuning_window import AdvancedTuningWindow


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PIDScope Offline")
        self.resize(1400, 900)
        
        self.dm = DeviceManager()
        self.current_channel = (1, 0)
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
        self.config = json.loads(cfg_path.read_text(encoding="utf-8"))
        self.schedule_bands = build_bands(self.config.get("gain_schedule", {}))

        self.worker = SerialWorker()
        
        # 重构：移除回调传参模式
        self.advanced_win = AdvancedTuningWindow(self.config)
        
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)

        self._build_ui()
        self._bind_events()

        # 重构：根据配置文件计算正确的帧间隔并刷新
        refresh_hz = self.config.get("ui_refresh_hz", 30)
        interval_ms = int(1000 / max(1, refresh_hz))
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_plot)
        self.timer.start(interval_ms)

    def _build_ui(self) -> None:
        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top_group = QVBoxLayout()

        conn_row = QHBoxLayout()
        self.port = QComboBox(); self.port.addItems(["COM3", "/dev/ttyUSB0", "/dev/ttyACM0"])
        self.port.setMinimumWidth(180)
        self.baud = QComboBox(); self.baud.addItems(["115200", "460800", "921600", "2000000", "3000000"])
        self.baud.setMinimumWidth(140)
        self.btn = QPushButton("Connect")
        self.btn.setMinimumWidth(110)
        self.status = QLabel("Idle")
        self.status.setMinimumWidth(320)
        conn_row.addWidget(QLabel("Serial Port"))
        conn_row.addWidget(self.port)
        conn_row.addWidget(QLabel("Baud"))
        conn_row.addWidget(self.baud)
        conn_row.addWidget(self.btn)
        conn_row.addStretch(1)
        conn_row.addWidget(QLabel("Status:"))
        conn_row.addWidget(self.status)

        tool_row = QHBoxLayout()
        self.advanced_btn = QPushButton("Advanced Tuning Lab")
        self.advanced_btn.setMinimumWidth(220)
        self.refresh_hint = QLabel("提示：高级功能与模拟均在 Advanced Tuning Lab 中，基础页保持最小化")
        tool_row.addWidget(self.advanced_btn)
        tool_row.addWidget(self.refresh_hint)
        tool_row.addStretch(1)

        top_group.addLayout(conn_row)
        top_group.addLayout(tool_row)
        layout.addLayout(top_group)

        mid = QHBoxLayout()
        self.device_list = QListWidget(); self.device_list.addItem("1:0")
        self.plot = pg.PlotWidget(background="#101418")
        self.plot.addLegend()
        self.curves = {k: self.plot.plot(pen=c, name=k) for k, c in [("target", "y"), ("feedback", "c"), ("error", "m"), ("output", "w")]}

        panel = QGroupBox("PID Params")
        form = QFormLayout(panel)
        self.kp = QDoubleSpinBox(); self.kp.setRange(0, 10000); self.kp.setDecimals(4)
        self.ki = QDoubleSpinBox(); self.ki.setRange(0, 10000); self.ki.setDecimals(4)
        self.kd = QDoubleSpinBox(); self.kd.setRange(0, 10000); self.kd.setDecimals(4)
        self.apply_btn = QPushButton("Apply")
        form.addRow("Kp", self.kp); form.addRow("Ki", self.ki); form.addRow("Kd", self.kd); form.addRow(self.apply_btn)

        mid.addWidget(self.device_list, 1)
        mid.addWidget(self.plot, 4)
        mid.addWidget(panel, 1)
        layout.addLayout(mid)

        self.analysis = QTextEdit(); self.analysis.setReadOnly(True)
        layout.addWidget(self.analysis)

    def _bind_events(self) -> None:
        self.thread.started.connect(lambda: self.worker.connect_port(self.port.currentText(), int(self.baud.currentText())))
        
        # 重构：监听解析完备的 Frame
        self.worker.frame_ready.connect(self.on_frame_ready)
        self.worker.status.connect(self.status.setText)
        self.btn.clicked.connect(self.toggle_connect)
        self.device_list.currentTextChanged.connect(self.on_select)
        self.advanced_btn.clicked.connect(self.advanced_win.show)
        
        # 重构：链接高级窗口的发送请求直接到 Worker 层
        self.advanced_win.send_frame_requested.connect(self.worker.send_bytes)

    def toggle_connect(self):
        if self.thread.isRunning():
            self.worker.stop(); self.thread.quit(); self.thread.wait(); self.btn.setText("Connect")
        else:
            self.thread.start(); self.btn.setText("Disconnect")

    def on_select(self, text: str):
        if not text:
            return
        d, c = text.split(":")
        self.current_channel = (int(d), int(c))

    def on_frame_ready(self, frame: PIDFrame):
        if frame.frame_type != FRAME_TELEMETRY:
            return
        try:
            tel = decode_telemetry(frame.payload)
            self.dm.push_telemetry(frame.device_id, frame.channel_id, tel)
            tag = f"{frame.device_id}:{frame.channel_id}"
            if not self.device_list.findItems(tag, Qt.MatchExactly):
                self.device_list.addItem(tag)
            self.advanced_win.update_from_telemetry(tel, frame.device_id, frame.channel_id)
        except Exception:
            pass

    def refresh_plot(self):
        d, c = self.current_channel
        window = self.dm.get_window(d, c, seconds=10)
        if len(window) < 2:
            return
        t0 = window[0]["timestamp_ms"]
        x = [(w["timestamp_ms"] - t0) / 1000.0 for w in window]
        for k in ["target", "feedback", "error", "output"]:
            self.curves[k].setData(x, [w[k] for w in window])
        result = basic_step_analysis(window)
        lines = [f"{k}: {v}" for k, v in result.items()]
        if self.schedule_bands:
            batt_v = float(window[-1].get("extra1", 0.0))
            pick = pick_gain_by_voltage(batt_v, self.schedule_bands)
            if pick:
                band, gains = pick
                lines.extend([
                    f"battery_voltage(extra1): {batt_v:.3f}V",
                    f"scheduled_band: {band}",
                    f"suggested_kp: {gains.kp}",
                    f"suggested_ki: {gains.ki}",
                    f"suggested_kd: {gains.kd}",
                ])
        self.analysis.setText("\n".join(lines))