import json
import math
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QComboBox, QFormLayout, QDoubleSpinBox, QGroupBox, QTextEdit,
    QFileDialog, QMessageBox, QLineEdit, QSplitter, QTabWidget, QTableWidget,
    QTableWidgetItem, QAbstractItemView, QCheckBox, QSpinBox
)
import pyqtgraph as pg

from transport.serial_worker import SerialWorker
from core.frame import PIDFrame
from core.protocol import (
    CAR_MODE_NAMES, FRAME_HEARTBEAT, FRAME_MAP_STATUS, FRAME_TELEMETRY, SEGMENT_NAMES,
    decode_heartbeat, decode_map_status, decode_telemetry, describe_status_flags,
)
from core.device_manager import DeviceManager
from core.command_router import CommandPlan, CommandRouter
from core.analyzer import basic_step_analysis
from core.gain_schedule import build_bands, pick_gain_by_voltage
from ui.advanced_tuning_window import AdvancedTuningWindow


class ConsoleWindow(QWidget):
    command_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("独立命令控制台")
        self.resize(760, 420)
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.input = QLineEdit()
        self.input.setPlaceholderText("输入命令后回车")
        self.send_btn = QPushButton("发送")
        self.log = QTextEdit(); self.log.setReadOnly(True)
        row.addWidget(self.input); row.addWidget(self.send_btn)
        layout.addLayout(row); layout.addWidget(self.log)
        self.send_btn.clicked.connect(self._send)
        self.input.returnPressed.connect(self._send)

    def _send(self):
        text = self.input.text().strip()
        if text:
            self.command_requested.emit(text + "\n")
            self.input.clear()

    def append_line(self, line: str):
        self.log.append(line)


class WaveWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("独立波形窗口")
        self.resize(900, 520)
        layout = QVBoxLayout(self)
        self.title = QLabel("等待数据")
        self.plot = pg.PlotWidget(background="#101418")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "时间", units="s")
        self.plot.addLegend(offset=(10, 10))
        self.curves = {
            "target": self.plot.plot(pen=pg.mkPen("#ffd43b", width=2), name="目标"),
            "feedback": self.plot.plot(pen=pg.mkPen("#22b8cf", width=3), name="反馈"),
            "error": self.plot.plot(pen=pg.mkPen("#da77f2", width=2), name="误差"),
            "output": self.plot.plot(pen=pg.mkPen("#f8f9fa", width=1), name="输出"),
        }
        layout.addWidget(self.title); layout.addWidget(self.plot)

    def update_wave(self, title: str, x, data: dict):
        self.title.setText(title)
        for key, curve in self.curves.items():
            curve.setData(x, data[key])


class MapWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("地图/轨迹窗口")
        self.resize(780, 560)
        self.last_distance: float | None = None
        self.x = 0.0
        self.y = 0.0
        self.points_x: list[float] = []
        self.points_y: list[float] = []
        layout = QVBoxLayout(self)
        self.state = QLabel("等待里程/yaw 遥测")
        self.map_state = QLabel("地图事件：等待 MAP_STATUS")
        self.plot = pg.PlotWidget(background="#101418")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "X", units="m")
        self.plot.setLabel("left", "Y", units="m")
        self.curve = self.plot.plot(pen=pg.mkPen("#51cf66", width=2), name="轨迹")
        self.pos = self.plot.plot(pen=None, symbol="o", symbolBrush="#ff922b", symbolSize=10)
        layout.addWidget(self.state); layout.addWidget(self.map_state); layout.addWidget(self.plot)

    def update_from_telemetry(self, tel: dict, status_text: str):
        distance = float(tel.get("extra1", 0.0))
        yaw = float(tel.get("extra2", 0.0))
        if self.last_distance is None:
            self.last_distance = distance
        delta = distance - self.last_distance
        self.last_distance = distance
        if 0.0 <= delta < 1.0:
            rad = math.radians(yaw)
            self.x += delta * math.cos(rad)
            self.y += delta * math.sin(rad)
            self.points_x.append(self.x)
            self.points_y.append(self.y)
            if len(self.points_x) > 3000:
                self.points_x = self.points_x[-3000:]
                self.points_y = self.points_y[-3000:]
        self.curve.setData(self.points_x, self.points_y)
        self.pos.setData([self.x], [self.y])
        self.state.setText(f"distance={distance:.3f}m yaw={yaw:.1f}deg  {status_text}")

    def update_map_status(self, status: dict, status_text: str):
        current_id = int(status.get("current_id", 0xFFFF))
        next_id = int(status.get("next_id", 0xFFFF))
        current_name = SEGMENT_NAMES.get(int(status.get("current_type", 0)), "UNKNOWN")
        next_name = SEGMENT_NAMES.get(int(status.get("next_type", 0)), "UNKNOWN")
        mode_name = CAR_MODE_NAMES.get(int(status.get("car_mode", 0)), "UNKNOWN")
        current_text = "无" if current_id == 0xFFFF else f"{current_id} {current_name}"
        next_text = "无" if next_id == 0xFFFF else f"{next_id} {next_name}"
        self.map_state.setText(
            f"模式={mode_name} 当前事件={current_text} 下一事件={next_text} "
            f"距下一事件={float(status.get('dist_to_next_m', -1.0)):.3f}m  {status_text}"
        )


class MainWindow(QMainWindow):
    remote_command_requested = Signal(str)
    connect_requested = Signal(str, int)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("智能车地面站 / PIDScope 离线版")
        self.resize(1400, 900)
        
        self.dm = DeviceManager()
        self.current_channel = (1, 1)
        self.latest_status_flags = 0
        self.latest_heartbeat_ms = 0
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
        self.config = json.loads(cfg_path.read_text(encoding="utf-8"))
        self.schedule_bands = build_bands(self.config.get("gain_schedule", {}))

        self.worker = SerialWorker()
        self.command_router = CommandRouter()
        self.channel_defs = {
            (1, 0): "循迹PID",
            (1, 1): "左轮速度PID",
            (1, 2): "右轮速度PID",
        }
        self.plots = {}
        self.curves_by_channel = {}
        self.pending_safety_feature = ""
        self.runtime_state = {
            "MOTOR_OUTPUT": "0",
            "SPEED_PID": "0",
            "PID_SCOPE": "0",
            "PID_SCOPE_OVER_HC05": "0",
            "REMOTE_CONTROL": "1",
            "MOTOR": "0",
            "SPIN_TEST": "0",
            "SPEED_TUNE": "0",
            "SPEED": "0.10",
            "TIMEOUT": "1000",
        }
        self.cycle_counts: dict[tuple[int, int], int] = {}
        self.feature_param_rows = {
            "MOTOR_OUTPUT": "电机输出",
            "SPEED_PID": "速度闭环",
            "TRACK_MAP_LEARNING": "地图学习",
            "PID_SCOPE": "PIDScope",
            "PID_SCOPE_OVER_HC05": "PIDScope走蓝牙",
            "OLED": "OLED显示",
            "LINE_SENSOR": "循迹传感器",
            "IMU": "IMU姿态",
            "ENCODER": "编码器",
            "MAP_PREDICTION": "地图预测",
            "LOST_PROTECTION": "丢线保护",
            "HC05_OLED_TEST": "蓝牙OLED测试",
            "HC05_RX_ECHO_TEST": "蓝牙回显测试",
            "REMOTE_CONTROL": "远程控制",
        }
        self.cfg_summary_aliases = {
            "REMOTE": "REMOTE_CONTROL",
            "MOTOR_OUT": "MOTOR_OUTPUT",
            "PID_HC05": "PID_SCOPE_OVER_HC05",
            "LINE": "LINE_SENSOR",
            "ENC": "ENCODER",
        }
        self.param_commands = [
            ("远控超时", "1000", "CFG TIMEOUT {value}", "远控超时时间，单位 ms，可运行期修改"),
            ("速度上限", "0.25", "CFG LIMIT {value}", "运行期速度上限，单位 m/s"),
            ("目标速度", "0.10", "CFG SPEED {value}", "运行期目标速度，单位 m/s"),
            ("运行模式", "TRACKING", "MODE {value}", "覆盖小车运行模式"),
            ("远控电机门", "0", "MOTOR {value}", "远控电机允许位，0/1"),
            ("循迹PID", "120 0 25", "PID LINE {value}", "循迹 PID：kp ki kd"),
            ("速度PID", "500 50 0", "PID SPEED {value}", "左右轮速度 PID：kp ki kd"),
            ("遥测周期", "150", "SCOPE PERIOD {value}", "PIDScope 遥测周期，单位 ms；高波特率可调低"),
            ("电机输出", "0", "CFG SET MOTOR_OUTPUT {value}", "安全开关：第一次点击预确认，第二次打开"),
            ("速度闭环", "0", "CFG SET SPEED_PID {value}", "安全开关：第一次点击预确认，第二次打开"),
            ("地图学习", "0", "CFG SET TRACK_MAP_LEARNING {value}", "安全开关：第一次点击预确认，第二次打开"),
            ("PIDScope", "0", "CFG SET PID_SCOPE {value}", "运行期开关"),
            ("PIDScope走蓝牙", "0", "CFG SET PID_SCOPE_OVER_HC05 {value}", "运行期开关"),
            ("OLED显示", "1", "CFG SET OLED {value}", "运行期开关"),
            ("循迹传感器", "1", "CFG SET LINE_SENSOR {value}", "运行期开关"),
            ("IMU姿态", "1", "CFG SET IMU {value}", "运行期开关"),
            ("编码器", "1", "CFG SET ENCODER {value}", "运行期开关"),
            ("地图预测", "0", "CFG SET MAP_PREDICTION {value}", "运行期开关"),
            ("丢线保护", "0", "CFG SET LOST_PROTECTION {value}", "运行期开关"),
            ("蓝牙OLED测试", "0", "CFG SET HC05_OLED_TEST {value}", "运行期开关"),
            ("蓝牙回显测试", "0", "CFG SET HC05_RX_ECHO_TEST {value}", "运行期开关"),
            ("远程控制", "1", "CFG SET REMOTE_CONTROL {value}", "运行期开关"),
        ]
        self.param_row_by_name: dict[str, int] = {}
        
        # 重构：移除回调传参模式
        self.advanced_win = AdvancedTuningWindow(self.config)
        self.console_win = ConsoleWindow()
        self.wave_win = WaveWindow()
        self.map_win = MapWindow()
        
        self.thread = QThread(self)
        self.worker.moveToThread(self.thread)

        self._build_ui()
        self._bind_events()
        self.update_button_states()

        # 重构：根据配置文件计算正确的帧间隔并刷新
        refresh_hz = self.config.get("ui_refresh_hz", 30)
        interval_ms = int(1000 / max(1, refresh_hz))
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_plot)
        self.timer.start(interval_ms)

        self.spin_test_timer = QTimer(self)
        self.spin_test_timer.setInterval(2000)
        self.spin_test_timer.timeout.connect(self.send_spin_test_heartbeat)

    def _build_ui(self) -> None:
        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top_group = QVBoxLayout()

        conn_row = QHBoxLayout()
        self.port = QComboBox(); self.port.setEditable(True)
        self.port.setMinimumWidth(360)
        self.refresh_ports_btn = QPushButton("刷新串口")
        self.refresh_ports_btn.setMinimumWidth(120)
        self.baud = QComboBox(); self.baud.addItems(["9600", "115200", "38400", "460800", "921600", "2000000", "3000000"])
        self.baud.setMinimumWidth(140)
        self.btn = QPushButton("连接")
        self.btn.setMinimumWidth(110)
        self.status = QLabel("空闲")
        self.status.setMinimumWidth(320)
        conn_row.addWidget(QLabel("串口/蓝牙端口"))
        conn_row.addWidget(self.port)
        conn_row.addWidget(self.refresh_ports_btn)
        conn_row.addWidget(QLabel("波特率"))
        conn_row.addWidget(self.baud)
        conn_row.addWidget(self.btn)
        conn_row.addStretch(1)
        conn_row.addWidget(QLabel("状态："))
        conn_row.addWidget(self.status)

        tool_row = QHBoxLayout()
        self.advanced_btn = QPushButton("高级调参实验室")
        self.advanced_btn.setMinimumWidth(220)
        self.console_win_btn = QPushButton("独立CMD")
        self.wave_win_btn = QPushButton("独立波形")
        self.map_win_btn = QPushButton("地图窗口")
        self.save_csv_btn = QPushButton("保存当前通道CSV")
        self.save_csv_btn.setMinimumWidth(170)
        self.refresh_hint = QLabel("提示：HC-05 配对后会显示为 COM/rfcomm 串口；当前固件默认 9600，若已改 AT+UART 则选择对应波特率")
        tool_row.addWidget(self.advanced_btn)
        tool_row.addWidget(self.console_win_btn)
        tool_row.addWidget(self.wave_win_btn)
        tool_row.addWidget(self.map_win_btn)
        tool_row.addWidget(self.save_csv_btn)
        tool_row.addWidget(self.refresh_hint)
        tool_row.addStretch(1)

        control_row = QHBoxLayout()
        test_row = QHBoxLayout()
        self.ping_btn = QPushButton("连通测试")
        self.stop_btn = QPushButton("停车")
        self.auto_btn = QPushButton("自动")
        self.motor_on_btn = QPushButton("电机开")
        self.motor_off_btn = QPushButton("电机关")
        self.scope_on_btn = QPushButton("开启波形")
        self.spin_test_btn = QPushButton("悬空持续转")
        self.speed_tune_btn = QPushButton("悬空闭环调参")
        self.step_sample_btn = QPushButton("半自动阶跃采样")
        self.spin_stop_btn = QPushButton("停止悬空测试")
        self.tune_target = QComboBox()
        self.tune_target.addItem("双轮速度", "BOTH")
        self.tune_target.addItem("左轮速度", "LEFT")
        self.tune_target.addItem("右轮速度", "RIGHT")
        self.tune_target_label = QLabel("调参目标")
        self.remote_speed = QDoubleSpinBox(); self.remote_speed.setRange(0.0, 1.0); self.remote_speed.setDecimals(3); self.remote_speed.setSingleStep(0.01); self.remote_speed.setValue(0.10)
        self.speed_btn = QPushButton("设置速度")
        self.motor_test_pwm = QSpinBox(); self.motor_test_pwm.setRange(0, 1000); self.motor_test_pwm.setValue(250); self.motor_test_pwm.setSingleStep(50)
        self.test_left_btn = QPushButton("左轮测试")
        self.test_right_btn = QPushButton("右轮测试")
        self.test_both_btn = QPushButton("双轮测试")
        self.test_stop_btn = QPushButton("停止测试")
        self.enc_query_btn = QPushButton("读取编码器")
        control_row.addWidget(QLabel("远程控制"))
        control_row.addWidget(self.ping_btn)
        control_row.addWidget(self.stop_btn)
        control_row.addWidget(self.auto_btn)
        control_row.addWidget(self.motor_on_btn)
        control_row.addWidget(self.motor_off_btn)
        control_row.addWidget(self.scope_on_btn)
        control_row.addWidget(self.spin_test_btn)
        control_row.addWidget(self.tune_target_label)
        control_row.addWidget(self.tune_target)
        control_row.addWidget(self.speed_tune_btn)
        control_row.addWidget(self.step_sample_btn)
        control_row.addWidget(self.spin_stop_btn)
        control_row.addWidget(QLabel("m/s"))
        control_row.addWidget(self.remote_speed)
        control_row.addWidget(self.speed_btn)
        control_row.addStretch(1)
        test_row.addWidget(QLabel("硬件自检 PWM"))
        test_row.addWidget(self.motor_test_pwm)
        test_row.addWidget(self.test_left_btn)
        test_row.addWidget(self.test_right_btn)
        test_row.addWidget(self.test_both_btn)
        test_row.addWidget(self.test_stop_btn)
        test_row.addWidget(self.enc_query_btn)
        test_row.addStretch(1)

        top_group.addLayout(conn_row)
        top_group.addLayout(tool_row)
        top_group.addLayout(control_row)
        top_group.addLayout(test_row)
        layout.addLayout(top_group)
        self.refresh_ports()

        content_splitter = QSplitter(Qt.Vertical)
        top_content = QWidget()
        top_content_layout = QVBoxLayout(top_content)

        mid = QHBoxLayout()
        self.plot_tabs = QTabWidget()
        for key, name in self.channel_defs.items():
            self.plot_tabs.addTab(self.create_channel_plot(key, name), f"{key[0]}:{key[1]} {name}")

        panel = QGroupBox("PID参数")
        form = QFormLayout(panel)
        self.kp = QDoubleSpinBox(); self.kp.setRange(0, 10000); self.kp.setDecimals(4)
        self.ki = QDoubleSpinBox(); self.ki.setRange(0, 10000); self.ki.setDecimals(4)
        self.kd = QDoubleSpinBox(); self.kd.setRange(0, 10000); self.kd.setDecimals(4)
        self.plot_samples = QSpinBox(); self.plot_samples.setRange(20, 2000); self.plot_samples.setValue(240)
        self.auto_cycle = QCheckBox("满样本自动计算并进入下一轮")
        self.auto_cycle.setChecked(False)
        self.cycle_samples = QSpinBox(); self.cycle_samples.setRange(10, 1000); self.cycle_samples.setValue(60)
        self.analyze_step_btn = QPushButton("阶跃分析")
        self.clear_plot_btn = QPushButton("清除当前波形")
        self.clear_all_plot_btn = QPushButton("清除全部波形")
        self.curve_checks = {}
        curve_row = QWidget(); curve_layout = QHBoxLayout(curve_row); curve_layout.setContentsMargins(0, 0, 0, 0)
        for key, text in [("target", "目标"), ("feedback", "反馈"), ("error", "误差"), ("output", "输出")]:
            cb = QCheckBox(text); cb.setChecked(True)
            cb.toggled.connect(lambda checked, k=key: self.set_curve_visible(k, checked))
            self.curve_checks[key] = cb
            curve_layout.addWidget(cb)
        curve_layout.addStretch(1)
        plot_btn_row = QWidget(); plot_btn_layout = QHBoxLayout(plot_btn_row); plot_btn_layout.setContentsMargins(0, 0, 0, 0)
        plot_btn_layout.addWidget(self.analyze_step_btn); plot_btn_layout.addWidget(self.clear_plot_btn); plot_btn_layout.addWidget(self.clear_all_plot_btn)
        self.channel_title = QLabel("1:1 左轮速度PID")
        self.channel_title.setStyleSheet("font-weight: 700; font-size: 14px;")
        self.channel_io = QLabel("目标/反馈：等待遥测")
        self.channel_error = QLabel("误差/输出：等待遥测")
        self.channel_gain = QLabel("PID参数：等待遥测")
        self.channel_state = QLabel("状态：等待遥测")
        for label in [self.channel_io, self.channel_error, self.channel_gain, self.channel_state]:
            label.setWordWrap(True)
        self.apply_btn = QPushButton("应用")
        form.addRow("Kp", self.kp); form.addRow("Ki", self.ki); form.addRow("Kd", self.kd)
        form.addRow("显示点数", self.plot_samples); form.addRow("循环样本", self.cycle_samples); form.addRow(self.auto_cycle)
        form.addRow("曲线", curve_row); form.addRow(plot_btn_row)
        form.addRow("通道", self.channel_title); form.addRow("目标/反馈", self.channel_io)
        form.addRow("误差/输出", self.channel_error); form.addRow("PID", self.channel_gain); form.addRow("状态", self.channel_state)
        form.addRow(self.apply_btn)

        mid.addWidget(self.plot_tabs, 4)
        mid.addWidget(panel, 1)
        top_content_layout.addLayout(mid)

        self.analysis = QTextEdit(); self.analysis.setReadOnly(True)
        top_content_layout.addWidget(self.analysis)
        content_splitter.addWidget(top_content)

        bottom_tabs = QTabWidget()
        bottom_tabs.addTab(self._build_console_tab(), "命令控制台")
        bottom_tabs.addTab(self._build_params_tab(), "地面站参数")
        content_splitter.addWidget(bottom_tabs)
        content_splitter.setStretchFactor(0, 4)
        content_splitter.setStretchFactor(1, 2)
        layout.addWidget(content_splitter)

    def _build_console_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        console_help = QLabel("常用命令：HELP、START、STOP、AUTO、MOTOR 0/1、MODE TRACKING、SPEED 0.15、LIMIT 0.25、PID LINE 120 0 25、STATUS、CFG LIST")
        console_row = QHBoxLayout()
        self.console_input = QLineEdit()
        self.console_input.setPlaceholderText("输入命令后按回车，例如：PID LINE 120 0 25")
        self.console_send_btn = QPushButton("发送")
        self.console_clear_btn = QPushButton("清空")
        self.console_log = QTextEdit(); self.console_log.setReadOnly(True)
        self.console_log.document().setMaximumBlockCount(300)
        console_row.addWidget(self.console_input)
        console_row.addWidget(self.console_send_btn)
        console_row.addWidget(self.console_clear_btn)
        layout.addWidget(console_help)
        layout.addLayout(console_row)
        layout.addWidget(self.console_log)
        return tab

    def _build_params_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        hint = QLabel("运行期参数会转换为 CMD 命令发送；编译能力开关不能在上位机中强行打开，可用 CFG ? / CFG LIST 查询当前状态。")
        btn_row = QHBoxLayout()
        self.param_refresh_btn = QPushButton("查询配置")
        self.param_apply_btn = QPushButton("应用选中项")
        btn_row.addWidget(self.param_refresh_btn)
        btn_row.addWidget(self.param_apply_btn)
        btn_row.addStretch(1)

        self.param_table = QTableWidget(len(self.param_commands), 4)
        self.param_table.setHorizontalHeaderLabels(["名称", "值", "命令", "说明"])
        self.param_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.param_table.setSelectionMode(QAbstractItemView.SingleSelection)
        for row, (name, value, command, desc) in enumerate(self.param_commands):
            self.param_row_by_name[name] = row
            self.param_table.setItem(row, 0, QTableWidgetItem(name))
            self.param_table.setItem(row, 1, QTableWidgetItem(value))
            self.param_table.setItem(row, 2, QTableWidgetItem(command))
            self.param_table.setItem(row, 3, QTableWidgetItem(desc))
        self.param_table.resizeColumnsToContents()

        layout.addWidget(hint)
        layout.addLayout(btn_row)
        layout.addWidget(self.param_table)
        return tab

    def _bind_events(self) -> None:
        self.thread.started.connect(self.start_worker_connection)
        self.connect_requested.connect(self.worker.connect_port)
        
        self.worker.status.connect(self.status.setText)
        self.worker.text_received.connect(self.append_console_line)
        self.btn.clicked.connect(self.toggle_connect)
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        self.plot_tabs.currentChanged.connect(self.on_plot_tab_changed)
        self.advanced_btn.clicked.connect(self.advanced_win.show)
        self.console_win_btn.clicked.connect(lambda: self.show_tool_window(self.console_win))
        self.wave_win_btn.clicked.connect(lambda: self.show_tool_window(self.wave_win))
        self.map_win_btn.clicked.connect(lambda: self.show_tool_window(self.map_win))
        self.save_csv_btn.clicked.connect(self.save_current_channel_csv)
        self.console_win.command_requested.connect(lambda text: self.send_remote_command(text))
        
        # Worker 读循环会占用子线程事件循环；发送走线程安全队列，避免 queued slot 堵住。
        self.advanced_win.send_frame_requested.connect(lambda data: self.worker.enqueue_bytes(data))
        self.remote_command_requested.connect(lambda text: self.worker.enqueue_text(text))
        self.ping_btn.clicked.connect(lambda: self.send_remote_command("$PING\n"))
        self.stop_btn.clicked.connect(lambda: self.send_remote_command("$STOP\n"))
        self.auto_btn.clicked.connect(lambda: self.send_remote_command("$AUTO\n"))
        self.motor_on_btn.clicked.connect(lambda: self.send_remote_command("$MOTOR,1\n"))
        self.motor_off_btn.clicked.connect(lambda: self.send_remote_command("$MOTOR,0\n"))
        self.scope_on_btn.clicked.connect(self.toggle_pid_scope_stream)
        self.spin_test_btn.clicked.connect(self.toggle_spin_test)
        self.speed_tune_btn.clicked.connect(self.toggle_speed_pid_tune)
        self.step_sample_btn.clicked.connect(self.start_semi_auto_step_sample)
        self.spin_stop_btn.clicked.connect(self.stop_spin_test)
        self.analyze_step_btn.clicked.connect(self.run_step_analysis)
        self.clear_plot_btn.clicked.connect(self.clear_current_plot)
        self.clear_all_plot_btn.clicked.connect(self.clear_all_plots)
        self.test_left_btn.clicked.connect(lambda: self.start_motor_test("LEFT"))
        self.test_right_btn.clicked.connect(lambda: self.start_motor_test("RIGHT"))
        self.test_both_btn.clicked.connect(lambda: self.start_motor_test("BOTH"))
        self.test_stop_btn.clicked.connect(self.stop_motor_test)
        self.enc_query_btn.clicked.connect(lambda: self.send_remote_command("ENC?\n"))
        self.speed_btn.clicked.connect(lambda: self.send_remote_command(f"$SPEED,{self.remote_speed.value():.3f}\n"))
        self.console_send_btn.clicked.connect(self.send_console_command)
        self.console_clear_btn.clicked.connect(self.console_log.clear)
        self.console_input.returnPressed.connect(self.send_console_command)
        self.param_refresh_btn.clicked.connect(lambda: self.send_remote_command("CFG LIST\n"))
        self.param_apply_btn.clicked.connect(self.apply_selected_param)

    def toggle_connect(self):
        if self.thread.isRunning():
            self.stop_worker_thread()
            self.btn.setText("连接")
            self.status.setText("已断开")
        else:
            self.btn.setText("断开")
            self.status.setText("正在连接...")
            self.thread.start(); self.btn.setText("断开")

    def stop_worker_thread(self):
        self.worker.stop()
        self.thread.quit()
        if not self.thread.wait(1500):
            self.thread.terminate()
            self.thread.wait(1000)

    def closeEvent(self, event):
        self.spin_test_timer.stop()
        self.timer.stop()
        for win in (self.advanced_win, self.console_win, self.wave_win, self.map_win):
            win.close()
        if self.thread.isRunning():
            self.stop_worker_thread()
        event.accept()

    def show_tool_window(self, window: QWidget):
        window.show()
        window.raise_()
        window.activateWindow()

    def start_worker_connection(self):
        self.connect_requested.emit(self.port.currentText(), int(self.baud.currentText()))

    def refresh_ports(self):
        current = self.port.currentText()
        self.port.blockSignals(True)
        self.port.clear()
        ports = SerialWorker.list_serial_ports()
        if ports:
            for device, label in ports:
                self.port.addItem(label, device)
        else:
            self.port.addItems(["COM3", "COM4", "/dev/rfcomm0", "/dev/ttyUSB0", "/dev/ttyACM0"])
        if current:
            index = self.port.findText(current)
            if index >= 0:
                self.port.setCurrentIndex(index)
            else:
                self.port.setEditText(current)
        self.port.blockSignals(False)

    def create_channel_plot(self, key: tuple[int, int], name: str):
        plot = pg.PlotWidget(background="#101418")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.setLabel("bottom", "时间", units="s")
        plot.setLabel("left", "数值")
        plot.setTitle(f"{key[0]}:{key[1]} {name}")
        plot.setClipToView(True)
        plot.setDownsampling(auto=True, mode="peak")
        plot.enableAutoRange(axis="y", enable=True)
        plot.addLegend(offset=(10, 10))
        plot.addLine(y=0, pen=pg.mkPen("#495057", width=1, style=Qt.DashLine))
        curves = {
            k: plot.plot(pen=pg.mkPen(c, width=width), name=label)
            for k, c, width, label in [
                ("target", "#ffd43b", 2, "目标"),
                ("feedback", "#22b8cf", 3, "反馈"),
                ("error", "#da77f2", 2, "误差"),
                ("output", "#f8f9fa", 1, "输出"),
            ]
        }
        self.plots[key] = plot
        self.curves_by_channel[key] = curves
        return plot

    def set_curve_visible(self, curve_key: str, visible: bool):
        for curves in self.curves_by_channel.values():
            if curve_key in curves:
                curves[curve_key].setVisible(visible)

    def on_plot_tab_changed(self, index: int):
        if index < 0:
            return
        key = list(self.channel_defs.keys())[index]
        self.select_channel(key[0], key[1])

    def send_remote_command(self, cmd: str):
        self.remote_command_requested.emit(cmd)
        self.apply_command_to_param_state(cmd)
        self.status.setText(f"已发送远控命令：{cmd.strip()}")

    def send_remote_commands(self, commands: list[str]):
        for cmd in commands:
            self.remote_command_requested.emit(cmd if cmd.endswith("\n") else cmd + "\n")
            self.apply_command_to_param_state(cmd)
        if commands:
            self.status.setText(f"已发送 {len(commands)} 条远控命令")

    def apply_command_plan(self, plan: CommandPlan):
        self.send_remote_commands(plan.commands)
        for key, value in plan.updates.items():
            self.update_runtime_state(key, value)
        if plan.status:
            self.status.setText(plan.status)

    def get_test_speed(self) -> float:
        speed = self.remote_speed.value()
        if speed <= 0.001:
            speed = 0.10
            self.remote_speed.setValue(speed)
            self.set_param_value("目标速度", f"{speed:.3f}")
        return speed

    def set_param_value(self, name: str, value: str):
        row = self.param_row_by_name.get(name)
        if row is None:
            return
        item = self.param_table.item(row, 1)
        if item:
            item.setText(value)
        if name == "目标速度" and hasattr(self, "remote_speed"):
            try:
                self.remote_speed.blockSignals(True)
                self.remote_speed.setValue(float(value))
            except ValueError:
                pass
            finally:
                self.remote_speed.blockSignals(False)

    def update_runtime_state(self, key: str, value: str):
        self.runtime_state[key] = value
        self.update_button_states()

    def update_button_states(self):
        pid_scope_on = self.runtime_state.get("PID_SCOPE") == "1" and self.runtime_state.get("PID_SCOPE_OVER_HC05") == "1"
        motor_on = self.runtime_state.get("MOTOR") == "1"
        spin_on = self.runtime_state.get("SPIN_TEST") == "1"
        speed_tune_on = self.runtime_state.get("SPEED_TUNE") == "1"
        motor_output_on = self.runtime_state.get("MOTOR_OUTPUT") == "1"

        self.scope_on_btn.setEnabled(not speed_tune_on)
        self.scope_on_btn.setText("关闭波形" if pid_scope_on else "开启波形")
        self.motor_on_btn.setEnabled(not motor_on and not spin_on and not speed_tune_on)
        self.motor_off_btn.setEnabled(motor_on and not spin_on and not speed_tune_on)
        self.motor_on_btn.setText("电机已开" if motor_on else "电机开")
        self.spin_test_btn.setEnabled(not speed_tune_on)
        self.speed_tune_btn.setEnabled(not spin_on)
        self.step_sample_btn.setEnabled(not spin_on and not speed_tune_on)
        self.tune_target.setEnabled(not spin_on and not speed_tune_on)
        self.spin_stop_btn.setEnabled(spin_on or speed_tune_on or motor_output_on or motor_on)
        self.spin_test_btn.setText("停止持续转" if spin_on and not speed_tune_on else "悬空持续转")
        self.speed_tune_btn.setText("停止闭环调参" if speed_tune_on else "悬空闭环调参")

    def set_feature_value(self, feature: str, value: str):
        name = self.feature_param_rows.get(feature)
        if name:
            self.set_param_value(name, value)
        self.update_runtime_state(feature, value)

    def apply_command_to_param_state(self, cmd: str):
        text = cmd.strip().lstrip("$").replace(",", " ")
        parts = text.split()
        if not parts:
            return
        if parts[0] == "SPEED" and len(parts) >= 2:
            self.set_param_value("目标速度", parts[1])
            self.update_runtime_state("SPEED", parts[1])
        elif parts[0] == "TUNE" and len(parts) >= 3:
            self.set_param_value("目标速度", parts[2])
            self.update_runtime_state("SPEED", parts[2])
        elif parts[0] == "MOTOR" and len(parts) >= 2:
            value = "1" if parts[1] in ("1", "ON") else "0"
            self.set_param_value("远控电机门", value)
            self.update_runtime_state("MOTOR", value)
        elif parts[0] == "MODE" and len(parts) >= 2:
            self.set_param_value("运行模式", parts[1])
        elif parts[0] == "STOP":
            self.set_param_value("远控电机门", "0")
            self.update_runtime_state("MOTOR", "0")
            self.update_runtime_state("SPIN_TEST", "0")
            self.update_runtime_state("SPEED_TUNE", "0")
        elif parts[:2] == ["CFG", "TIMEOUT"] and len(parts) >= 3:
            self.set_param_value("远控超时", parts[2])
            self.update_runtime_state("TIMEOUT", parts[2])
        elif parts[:2] in (["CFG", "LIMIT"], ["CFG", "MAXSPEED"]) and len(parts) >= 3:
            self.set_param_value("速度上限", parts[2])
        elif parts[:2] == ["CFG", "SPEED"] and len(parts) >= 3:
            self.set_param_value("目标速度", parts[2])
            self.update_runtime_state("SPEED", parts[2])
            try:
                self.remote_speed.setValue(float(parts[2]))
            except ValueError:
                pass
        elif parts[:2] == ["SCOPE", "PERIOD"] and len(parts) >= 3:
            self.set_param_value("遥测周期", parts[2])
        elif parts[:2] == ["CFG", "SET"] and len(parts) >= 4:
            self.set_feature_value(parts[2], "1" if parts[3] in ("1", "ON") else "0")

    def toggle_pid_scope_stream(self):
        self.apply_command_plan(self.command_router.toggle_pid_scope_stream(self.runtime_state, self.current_channel))

    def toggle_spin_test(self):
        if self.runtime_state.get("SPIN_TEST") == "1" and self.runtime_state.get("SPEED_TUNE") != "1":
            self.stop_spin_test()
            return
        self.start_spin_test()

    def start_spin_test(self):
        speed = self.get_test_speed()
        self.apply_command_plan(self.command_router.start_spin_test(speed))
        self.spin_test_timer.start()

    def toggle_speed_pid_tune(self):
        if self.runtime_state.get("SPEED_TUNE") == "1":
            self.stop_spin_test()
            return
        self.start_speed_pid_tune()

    def start_speed_pid_tune(self):
        speed = self.get_test_speed()
        target = self.tune_target.currentData() or "BOTH"
        self.apply_command_plan(self.command_router.start_speed_pid_tune(speed, target))
        if target == "RIGHT":
            self.select_channel(1, 2)
        elif target == "LEFT":
            self.select_channel(1, 1)
        else:
            self.select_channel(1, 1)
        self.spin_test_timer.start()
        self.status.setText(f"悬空闭环调参已启动：目标={self.tune_target.currentText()}，已打开速度PID和PIDScope，先确认编码器反馈方向正确。")

    def start_semi_auto_step_sample(self):
        target = self.tune_target.currentData() or "LEFT"
        if target == "BOTH":
            target = "LEFT"
            self.tune_target.setCurrentIndex(max(0, self.tune_target.findData("LEFT")))
        channel_id = 2 if target == "RIGHT" else 1
        self.select_channel(1, channel_id)
        self.clear_current_plot()
        self.apply_command_plan(self.command_router.prepare_step_sample(target))
        self.spin_test_timer.start()
        QTimer.singleShot(800, lambda t=target: self.fire_semi_auto_step(t))

    def fire_semi_auto_step(self, target: str):
        if self.runtime_state.get("SPEED_TUNE") != "1":
            return
        speed = self.get_test_speed()
        self.apply_command_plan(self.command_router.fire_step_sample(target, speed))

    def send_spin_test_heartbeat(self):
        self.worker.enqueue_text("START\n", monitor=False)

    def start_motor_test(self, mode: str):
        pwm = self.motor_test_pwm.value()
        self.apply_command_plan(self.command_router.start_motor_test(mode, pwm))
        self.spin_test_timer.start()

    def stop_motor_test(self):
        self.spin_test_timer.stop()
        self.apply_command_plan(self.command_router.stop_motor_test())

    def stop_spin_test(self):
        self.spin_test_timer.stop()
        self.apply_command_plan(self.command_router.stop_spin_test())

    def clear_current_plot(self):
        d, c = self.current_channel
        self.dm.clear_channel(d, c)
        self.cycle_counts[(d, c)] = 0
        for curve in self.curves_by_channel.get((d, c), {}).values():
            curve.setData([])
        self.analysis.clear()
        self.channel_io.setText("目标/反馈：已清除")
        self.channel_error.setText("误差/输出：已清除")
        self.channel_gain.setText("PID参数：已清除")
        self.channel_state.setText("状态：已清除当前通道波形")

    def clear_all_plots(self):
        self.dm.clear_all()
        self.cycle_counts.clear()
        for curves in self.curves_by_channel.values():
            for curve in curves.values():
                curve.setData([])
        self.analysis.clear()
        self.channel_io.setText("目标/反馈：已清除")
        self.channel_error.setText("误差/输出：已清除")
        self.channel_gain.setText("PID参数：已清除")
        self.channel_state.setText("状态：已清除全部通道波形")

    def send_console_command(self):
        cmd = self.console_input.text().strip()
        if not cmd:
            return
        self.remote_command_requested.emit(cmd + "\n")
        self.console_input.clear()

    def append_console_line(self, line: str):
        self.update_params_from_reply(line)
        self.console_win.append_line(line)
        if line.startswith(">"):
            self.console_log.setAlignment(Qt.AlignRight)
        else:
            self.console_log.setAlignment(Qt.AlignLeft)
        self.console_log.append(line)

    def update_params_from_reply(self, line: str):
        if line.startswith("$OK,SCOPE,PERIOD,"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 4:
                self.set_param_value("遥测周期", parts[3])
            return
        if line.startswith("$OK,TUNE,"):
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 5 and parts[3] == "SPEED":
                self.set_param_value("目标速度", parts[4])
            return
        if not line.startswith("$CFG") and not line.startswith("$STATUS"):
            return
        parts = [p.strip() for p in line.split(",")]
        if len(parts) >= 5 and parts[:2] == ["$CFG", "FEATURE"]:
            self.set_feature_value(parts[2], parts[3])
            return
        if parts and parts[0] == "$CFG":
            for i in range(1, len(parts) - 1, 2):
                key, value = parts[i], parts[i + 1]
                feature = self.cfg_summary_aliases.get(key, key)
                if feature in self.feature_param_rows:
                    self.set_feature_value(feature, value)
                elif key == "TIMEOUT":
                    self.set_param_value("远控超时", value)
                elif key == "LIMIT":
                    self.set_param_value("速度上限", value)
                elif key == "SPEED":
                    self.set_param_value("目标速度", value)
        elif parts and parts[0] == "$STATUS":
            for i in range(1, len(parts) - 1, 2):
                key, value = parts[i], parts[i + 1]
                if key == "MOTOR":
                    self.set_param_value("远控电机门", value)
                    self.update_runtime_state("MOTOR", value)
                elif key == "SPEED":
                    self.set_param_value("目标速度", value)
                elif key == "LIMIT":
                    self.set_param_value("速度上限", value)
                elif key == "TIMEOUT":
                    self.set_param_value("远控超时", value)

    def apply_selected_param(self):
        row = self.param_table.currentRow()
        if row < 0:
            return
        value_item = self.param_table.item(row, 1)
        command_item = self.param_table.item(row, 2)
        if not value_item or not command_item:
            return
        value = value_item.text().strip()
        command = command_item.text().strip()
        if "{value}" not in command:
            self.send_remote_command(command + "\n")
            return
        if not value or value.lower() == "read only":
            self.status.setText("选中的参数是只读项")
            return
        final_command = command.replace("{value}", value)
        parts = final_command.split()
        safety_features = {"MOTOR_OUTPUT", "SPEED_PID", "TRACK_MAP_LEARNING"}
        if len(parts) >= 4 and parts[:2] == ["CFG", "SET"] and parts[2] in safety_features and parts[3] not in ("0", "OFF"):
            feature = parts[2]
            if self.pending_safety_feature != feature:
                self.pending_safety_feature = feature
                self.send_remote_command(f"CFG ARM {feature}\n")
                self.status.setText(f"安全开关 {feature} 已预确认，请在5秒内再次点击“应用选中项”打开。")
                return
            self.pending_safety_feature = ""
        self.send_remote_command(final_command + "\n")

    def save_current_channel_csv(self):
        d, c = self.current_channel
        default_name = f"pidscope_device{d}_ch{c}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "保存当前通道CSV", default_name, "CSV文件 (*.csv)")
        if not path:
            return
        count = self.dm.export_channel_csv(d, c, path)
        if count == 0:
            QMessageBox.information(self, "保存CSV", "当前通道没有可保存的遥测数据。")
            return
        self.status.setText(f"已保存 {count} 行遥测数据到 {path}")
        QMessageBox.information(self, "保存CSV", f"已保存 {count} 行遥测数据。")

    def select_channel(self, device_id: int, channel_id: int):
        self.current_channel = (device_id, channel_id)
        tag = f"{device_id}:{channel_id}"
        keys = list(self.channel_defs.keys())
        if (device_id, channel_id) in keys:
            self.plot_tabs.blockSignals(True)
            self.plot_tabs.setCurrentIndex(keys.index((device_id, channel_id)))
            self.plot_tabs.blockSignals(False)
        self.channel_title.setText(f"{tag} {self.channel_name(device_id, channel_id)}")
        self.send_remote_command(f"SCOPE {device_id} {channel_id}\n")
        self.refresh_channel_plot(device_id, channel_id)

    def channel_name(self, device_id: int, channel_id: int) -> str:
        names = {0: "循迹PID", 1: "左轮速度PID", 2: "右轮速度PID"}
        return names.get(channel_id, f"通道{channel_id}")

    def on_frame_ready(self, frame: PIDFrame, update_advanced: bool = True):
        if frame.frame_type == FRAME_HEARTBEAT:
            try:
                hb = decode_heartbeat(frame.payload)
                self.latest_heartbeat_ms = int(hb["uptime_ms"])
                self.latest_status_flags = int(hb["status_flags"])
                self.status.setText(f"心跳 {self.latest_heartbeat_ms} ms | {'/'.join(describe_status_flags(self.latest_status_flags))}")
            except Exception:
                pass
            return None
        if frame.frame_type == FRAME_MAP_STATUS:
            try:
                map_status = decode_map_status(frame.payload)
                self.latest_status_flags = int(map_status.get("status_flags", self.latest_status_flags))
                if self.map_win.isVisible():
                    self.map_win.update_map_status(map_status, "/".join(describe_status_flags(self.latest_status_flags)))
            except Exception:
                pass
            return None
        if frame.frame_type != FRAME_TELEMETRY:
            return None
        try:
            tel = decode_telemetry(frame.payload)
            self.latest_status_flags = int(tel.get("status_flags", self.latest_status_flags))
            self.dm.push_telemetry(frame.device_id, frame.channel_id, tel)
            if update_advanced:
                self.advanced_win.update_from_telemetry(tel, frame.device_id, frame.channel_id)
            if update_advanced and self.map_win.isVisible():
                self.map_win.update_from_telemetry(tel, "/".join(describe_status_flags(self.latest_status_flags)))
            return tel
        except Exception:
            return None

    def drain_serial_frames(self) -> int:
        frames = self.worker.drain_frames()
        latest_by_channel: dict[tuple[int, int], dict] = {}
        for frame in frames:
            tel = self.on_frame_ready(frame, update_advanced=False)
            if tel is not None:
                latest_by_channel[(frame.device_id, frame.channel_id)] = tel
        for (device_id, channel_id), tel in latest_by_channel.items():
            self.advanced_win.update_from_telemetry(tel, device_id, channel_id)
        if self.map_win.isVisible() and latest_by_channel:
            tel = latest_by_channel.get(self.current_channel) or next(iter(latest_by_channel.values()))
            self.map_win.update_from_telemetry(tel, "/".join(describe_status_flags(self.latest_status_flags)))
        if self.worker.dropped_frames:
            self.status.setText(f"遥测过载：已丢弃 {self.worker.dropped_frames} 帧，UI 仍按固定帧率刷新")
            self.worker.dropped_frames = 0
        return len(frames)

    def refresh_plot(self):
        self.drain_serial_frames()
        self.refresh_channel_plot(*self.current_channel)

    def refresh_channel_plot(self, d: int, c: int):
        data = self.dm.get_latest_arrays(d, c, count=int(self.plot_samples.value()))
        sample_count = len(data["timestamp_ms"])
        if sample_count < 2:
            if (d, c) == self.current_channel:
                self.channel_state.setText(f"状态：等待更多样本，当前 {sample_count} 个")
            return
        x = data["timestamp_ms"] / 1000.0
        x = x - x[0]
        curves = self.curves_by_channel.get((d, c), {})
        for k in ["target", "feedback", "error", "output"]:
            if k in curves:
                curves[k].setData(x, data[k])
        if self.wave_win.isVisible():
            self.wave_win.update_wave(f"{d}:{c} {self.channel_name(d, c)}", x, data)
        if (d, c) != self.current_channel:
            return
        last = {name: values[-1] for name, values in data.items()}
        self.channel_io.setText(f"目标：{last['target']:.3f}\n反馈：{last['feedback']:.3f}")
        self.channel_error.setText(f"误差：{last['error']:.3f}\n输出：{last['output']:.1f}")
        self.channel_gain.setText(f"Kp：{last['kp']:.3f}\nKi：{last['ki']:.3f}\nKd：{last['kd']:.3f}")
        total_rows = len(self.dm.get_rows(d, c))
        flags_text = "/".join(describe_status_flags(int(last.get("status_flags", 0))))
        self.channel_state.setText(f"窗口样本：{sample_count}\n累计样本：{total_rows}\n时间：{int(last['timestamp_ms'])} ms\n状态：{flags_text}")
        if self.schedule_bands:
            batt_v = float(last.get("extra1", 0.0))
            pick = pick_gain_by_voltage(batt_v, self.schedule_bands)
            if pick:
                band, gains = pick
                lines = [
                    f"电池电压(extra1): {batt_v:.3f}V",
                    f"调度分段: {band}",
                    f"建议Kp: {gains.kp}",
                    f"建议Ki: {gains.ki}",
                    f"建议Kd: {gains.kd}",
                ]
                if not self.analysis.toPlainText().startswith("手动阶跃分析"):
                    self.analysis.setText("\n".join(lines))

        if self.auto_cycle.isChecked():
            rows = self.dm.get_rows(d, c)
            target_count = int(self.cycle_samples.value())
            if len(rows) >= target_count:
                cycle_no = self.cycle_counts.get((d, c), 0) + 1
                self.cycle_counts[(d, c)] = cycle_no
                batch = rows[:target_count]
                batch_result = basic_step_analysis(batch)
                labels = {
                    "status": "状态", "step_size": "阶跃幅度", "overshoot_pct": "超调(%)",
                    "steady_state_error": "稳态误差", "oscillation_count": "振荡次数", "iae": "误差积分IAE",
                }
                batch_lines = [f"第 {cycle_no} 轮自动计算完成：{d}:{c} {self.channel_name(d, c)}"]
                batch_lines.extend([f"{labels.get(k, k)}: {v}" for k, v in batch_result.items()])
                batch_lines.append("已清空当前通道缓存，开始采集下一轮。")
                self.analysis.setText("\n".join(batch_lines))
                self.dm.clear_channel(d, c)
                for curve in self.curves_by_channel.get((d, c), {}).values():
                    curve.setData([])
                self.channel_state.setText(f"状态：第 {cycle_no} 轮完成，下一轮采集中")

    def run_step_analysis(self):
        d, c = self.current_channel
        window = self.dm.get_latest(d, c, count=int(self.plot_samples.value()))
        result = basic_step_analysis(window)
        labels = {
            "status": "状态", "step_size": "阶跃幅度", "overshoot_pct": "超调(%)",
            "steady_state_error": "稳态误差", "oscillation_count": "振荡次数", "iae": "误差积分IAE",
        }
        lines = [f"手动阶跃分析：{d}:{c} {self.channel_name(d, c)}"]
        lines.extend(f"{labels.get(k, k)}: {v}" for k, v in result.items())
        advice = self.pid_tuning_advice(window, result)
        if advice:
            lines.append("")
            lines.extend(advice)
        self.analysis.setText("\n".join(lines))

    def pid_tuning_advice(self, window: list[dict], result: dict) -> list[str]:
        if result.get("status") != "ok" or not window:
            return ["PID建议：数据不足，先执行半自动阶跃采样并等待 3-5 秒。"]
        last = window[-1]
        kp = float(last.get("kp", 0.0))
        ki = float(last.get("ki", 0.0))
        kd = float(last.get("kd", 0.0))
        overshoot = float(result.get("overshoot_pct", 0.0))
        steady_error = float(result.get("steady_state_error", 0.0))
        oscillations = int(result.get("oscillation_count", 0))
        step_size = abs(float(result.get("step_size", 0.0)))

        next_kp, next_ki, next_kd = kp, ki, kd
        reasons: list[str] = []
        if overshoot > 20.0 or oscillations >= 6:
            next_kp *= 0.90
            next_ki *= 0.90
            next_kd = kd * 1.05 if kd > 0.0 else max(kp * 0.002, 0.001)
            reasons.append("超调或振荡偏大：建议降低 Kp/Ki，并小幅提高 Kd。")
        elif steady_error > max(0.02, step_size * 0.08):
            next_ki = ki * 1.10 if ki > 0.0 else max(kp * 0.02, 0.001)
            reasons.append("稳态误差偏大：建议小幅提高 Ki。")
        elif overshoot < 5.0 and oscillations <= 2:
            next_kp = kp * 1.05 if kp > 0.0 else 1.0
            reasons.append("响应较稳：可以小幅提高 Kp 以加快响应。")
        else:
            reasons.append("响应可接受：建议暂不自动改参，继续单轮验证。")

        return [
            "PID建议：只给建议，不自动下发。",
            *reasons,
            f"当前 PID：Kp={kp:.4g}, Ki={ki:.4g}, Kd={kd:.4g}",
            f"建议 PID：Kp={next_kp:.4g}, Ki={next_ki:.4g}, Kd={next_kd:.4g}",
        ]
