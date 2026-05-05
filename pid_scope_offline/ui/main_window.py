from PySide6.QtCore import QThread, QTimer, Qt
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QComboBox, QListWidget, QFormLayout, QDoubleSpinBox, QGroupBox, QTextEdit
)
import pyqtgraph as pg

from transport.serial_worker import SerialWorker
from core.parser import FrameParser
from core.protocol import FRAME_TELEMETRY, decode_telemetry
from core.device_manager import DeviceManager
from core.analyzer import basic_step_analysis


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("PIDScope Offline")
        self.resize(1400, 900)
        self.parser = FrameParser()
        self.dm = DeviceManager()
        self.current_channel = (1, 0)

        self.worker = SerialWorker()
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)

        self._build_ui()
        self._bind_events()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_plot)
        self.timer.start(33)

    def _build_ui(self) -> None:
        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.port = QComboBox(); self.port.addItems(["COM3", "/dev/ttyUSB0", "/dev/ttyACM0"])
        self.baud = QComboBox(); self.baud.addItems(["115200", "460800", "921600", "2000000", "3000000"])
        self.btn = QPushButton("Connect")
        self.status = QLabel("Idle")
        for w in [QLabel("Port"), self.port, QLabel("Baud"), self.baud, self.btn, self.status]: top.addWidget(w)
        top.addStretch(1)
        layout.addLayout(top)

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
        self.worker.data_ready.connect(self.on_data)
        self.worker.status.connect(self.status.setText)
        self.btn.clicked.connect(self.toggle_connect)
        self.device_list.currentTextChanged.connect(self.on_select)

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

    def on_data(self, raw: bytes):
        for frame in self.parser.feed(raw):
            if frame.frame_type != FRAME_TELEMETRY:
                continue
            try:
                tel = decode_telemetry(frame.payload)
                self.dm.push_telemetry(frame.device_id, frame.channel_id, tel)
                tag = f"{frame.device_id}:{frame.channel_id}"
                if not self.device_list.findItems(tag, Qt.MatchExactly):
                    self.device_list.addItem(tag)
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
        self.analysis.setText("\n".join(f"{k}: {v}" for k, v in result.items()))
