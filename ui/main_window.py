import json
import math
import time
from pathlib import Path

from PySide6.QtCore import QThread, QTimer, Qt, Signal
from PySide6.QtGui import QColor
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
from core.analyzer import basic_step_analysis, identify_motor_step_model
from core.device_schema import load_default_schema
from core.gain_schedule import build_bands, pick_gain_by_voltage
from ui.advanced_tuning_window import AdvancedTuningWindow
from ui.device_config_window import DeviceConfigWindow


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
        self.plot.setLabel("left", "目标/反馈", units="m/s")
        self.plot.showAxis("right")
        self.plot.getAxis("right").setLabel("输出", units="PWM")
        self.output_view = pg.ViewBox()
        self.plot.scene().addItem(self.output_view)
        self.plot.getAxis("right").linkToView(self.output_view)
        self.output_view.setXLink(self.plot)
        self.output_view.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

        def update_output_view():
            self.output_view.setGeometry(self.plot.getViewBox().sceneBoundingRect())
            self.output_view.linkedViewChanged(self.plot.getViewBox(), self.output_view.XAxis)

        self.plot.getViewBox().sigResized.connect(update_output_view)
        legend = self.plot.addLegend(offset=(10, 10))
        self.curves = {
            "target": self.plot.plot(pen=pg.mkPen("#ffd43b", width=2), name="目标"),
            "feedback": self.plot.plot(pen=pg.mkPen("#22b8cf", width=3), name="反馈"),
            "error": self.plot.plot(pen=pg.mkPen("#da77f2", width=2), name="误差"),
        }
        self.curves["output"] = pg.PlotDataItem(pen=pg.mkPen("#f8f9fa", width=1), name="输出")
        self.output_view.addItem(self.curves["output"])
        legend.addItem(self.curves["output"], "输出")
        update_output_view()
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
        self.latest_map_status: dict | None = None
        cfg_path = Path(__file__).resolve().parents[1] / "config" / "default_config.json"
        self.config = json.loads(cfg_path.read_text(encoding="utf-8"))
        self.device_schema = load_default_schema()
        self.schedule_bands = build_bands(self.config.get("gain_schedule", {}))

        self.worker = SerialWorker()
        self.command_router = CommandRouter()
        self.channel_defs = self.device_schema.channel_defs
        if self.channel_defs:
            self.current_channel = next(iter(self.channel_defs.keys()))
        self.plots = {}
        self.curves_by_channel = {}
        self.pending_safety_feature = ""
        self._highlighted_row: tuple[QTableWidget, int] | None = None
        self._safety_confirm_timer = QTimer(self)
        self._safety_confirm_timer.setSingleShot(True)
        self._safety_confirm_timer.timeout.connect(self._clear_safety_highlight)
        self._last_cmd_send_s: dict[str, float] = {}
        self._cmd_send_min_interval_s = 0.15
        self.runtime_state = {
            "MOTOR_OUTPUT": "0",
            "SPEED_PID": "0",
            "PID_SCOPE": "0",
            "PID_SCOPE_OVER_HC05": "0",
            "REMOTE_CONTROL": "0",
            "FLASH_STORAGE": "0",
            "MOTOR": "0",
            "SPIN_TEST": "0",
            "SPEED_TUNE": "0",
            "SPEED": "0.225",
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
            "FLASH_STORAGE": "Flash存储",
        }
        self.cfg_summary_aliases = {
            "REMOTE": "REMOTE_CONTROL",
            "MOTOR_OUT": "MOTOR_OUTPUT",
            "PID_HC05": "PID_SCOPE_OVER_HC05",
            "FLASH": "FLASH_STORAGE",
            "LINE": "LINE_SENSOR",
            "ENC": "ENCODER",
        }
        self.param_commands = self.device_schema.param_commands
        self.param_row_by_name: dict[str, tuple[QTableWidget, int]] = {}
        self.dirty_pid_params: set[str] = set()
        self.pending_pid_values_by_channel: dict[int, str] = {}
        self.latest_model_recommendation: dict | None = None
        self.param_specs_by_name = {param.name: param for param in self.device_schema.ground_params}
        
        # 重构：移除回调传参模式
        self.advanced_win = AdvancedTuningWindow(self.config)
        self.console_win = ConsoleWindow()
        self.wave_win = WaveWindow()
        self.map_win = MapWindow()
        self.device_config_win = DeviceConfigWindow()
        
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
        baud_index = self.baud.findText(str(self.device_schema.default_baud))
        if baud_index >= 0:
            self.baud.setCurrentIndex(baud_index)
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
        self.device_config_btn = QPushButton("设备配置/生成驱动")
        self.console_win_btn = QPushButton("独立CMD")
        self.wave_win_btn = QPushButton("独立波形")
        self.map_win_btn = QPushButton("地图窗口")
        self.save_csv_btn = QPushButton("保存当前通道CSV")
        self.save_csv_btn.setMinimumWidth(170)
        self.save_all_csv_btn = QPushButton("保存全部通道CSV")
        self.save_all_csv_btn.setMinimumWidth(170)
        self.refresh_hint = QLabel("提示：HC-05 配对后会显示为 COM/rfcomm 串口；当前固件默认 9600，若已改 AT+UART 则选择对应波特率")
        tool_row.addWidget(self.advanced_btn)
        tool_row.addWidget(self.device_config_btn)
        tool_row.addWidget(self.console_win_btn)
        tool_row.addWidget(self.wave_win_btn)
        tool_row.addWidget(self.map_win_btn)
        tool_row.addWidget(self.save_csv_btn)
        tool_row.addWidget(self.save_all_csv_btn)
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
        self.straight_test_btn = QPushButton("长直线测试")
        self.race_track_btn = QPushButton("一键启动循迹")
        self.spin_test_btn = QPushButton("悬空持续转")
        self.speed_tune_btn = QPushButton("悬空闭环调参")
        self.step_sample_btn = QPushButton("半自动阶跃采样")
        self.spin_stop_btn = QPushButton("停止悬空测试")
        self.tune_target = QComboBox()
        self.tune_target.addItem("双轮速度", "BOTH")
        self.tune_target.addItem("左轮速度", "LEFT")
        self.tune_target.addItem("右轮速度", "RIGHT")
        self.tune_target_label = QLabel("调参目标")
        self.remote_speed = QDoubleSpinBox(); self.remote_speed.setRange(0.0, 1.0); self.remote_speed.setDecimals(3); self.remote_speed.setSingleStep(0.01); self.remote_speed.setValue(0.225)
        self.speed_btn = QPushButton("设置速度")
        self.motor_test_pwm = QSpinBox(); self.motor_test_pwm.setRange(0, 7100); self.motor_test_pwm.setValue(1200); self.motor_test_pwm.setSingleStep(100)
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
        control_row.addWidget(self.straight_test_btn)
        control_row.addWidget(self.race_track_btn)
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

        panel = QGroupBox("波形状态")
        form = QFormLayout(panel)
        self.plot_samples = QSpinBox(); self.plot_samples.setRange(20, 2000); self.plot_samples.setValue(240)
        self.auto_cycle = QCheckBox("满样本自动计算并进入下一轮")
        self.auto_cycle.setChecked(False)
        self.cycle_samples = QSpinBox(); self.cycle_samples.setRange(10, 1000); self.cycle_samples.setValue(60)
        self.analyze_step_btn = QPushButton("阶跃分析")
        self.apply_model_btn = QPushButton("应用模型建议")
        self.apply_model_btn.setEnabled(False)
        self.clear_plot_btn = QPushButton("清除当前波形")
        self.clear_all_plot_btn = QPushButton("清除全部波形")
        self.curve_checks = {}
        curve_row = QWidget(); curve_layout = QHBoxLayout(curve_row); curve_layout.setContentsMargins(0, 0, 0, 0)
        for key, text in [("target", "目标"), ("feedback", "实际速度"), ("error", "误差"), ("output", "输出PWM")]:
            cb = QCheckBox(text); cb.setChecked(key != "output")
            cb.toggled.connect(lambda checked, k=key: self.set_curve_visible(k, checked))
            self.curve_checks[key] = cb
            curve_layout.addWidget(cb)
        curve_layout.addStretch(1)
        plot_btn_row = QWidget(); plot_btn_layout = QHBoxLayout(plot_btn_row); plot_btn_layout.setContentsMargins(0, 0, 0, 0)
        plot_btn_layout.addWidget(self.analyze_step_btn); plot_btn_layout.addWidget(self.apply_model_btn); plot_btn_layout.addWidget(self.clear_plot_btn); plot_btn_layout.addWidget(self.clear_all_plot_btn)
        init_d, init_c = self.current_channel
        self.channel_title = QLabel(f"{init_d}:{init_c} {self.channel_name(init_d, init_c)}")
        self.channel_title.setStyleSheet("font-weight: 700; font-size: 14px;")
        self.channel_io = QLabel("目标/实际速度：等待遥测")
        self.channel_error = QLabel("误差/输出PWM：等待遥测")
        self.channel_gain = QLabel("PID参数：等待遥测")
        self.channel_state = QLabel("状态：等待遥测")
        for label in [self.channel_io, self.channel_error, self.channel_gain, self.channel_state]:
            label.setWordWrap(True)
        form.addRow("显示点数", self.plot_samples); form.addRow("循环样本", self.cycle_samples); form.addRow(self.auto_cycle)
        form.addRow("曲线", curve_row); form.addRow(plot_btn_row)
        form.addRow("通道", self.channel_title); form.addRow("目标/实际速度", self.channel_io)
        form.addRow("误差/输出PWM", self.channel_error); form.addRow("PID", self.channel_gain); form.addRow("状态", self.channel_state)

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

        console_help = QLabel("常用命令：HELP、START、STOP、AUTO、MOTOR 0/1、MODE TRACKING、SPEED 0.225、LIMIT 0.225、PID LINE 60 0 0.25、STATUS、CFG LIST")
        console_row = QHBoxLayout()
        self.console_input = QLineEdit()
        self.console_input.setPlaceholderText("输入命令后按回车，例如：PID LINE 60 0 0.25")
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
        self.flash_enable_btn = QPushButton("启用Flash功能")
        self.params_query_btn = QPushButton("查看当前参数")
        self.flash_query_btn = QPushButton("查看Flash内容")
        self.flash_save_pid_btn = QPushButton("保存PID")
        self.flash_save_ff_btn = QPushButton("保存前馈")
        self.flash_save_map_btn = QPushButton("保存地图")
        self.flash_save_btn = QPushButton("保存全部")
        self.flash_load_btn = QPushButton("从Flash加载")
        self.flash_erase_pid_btn = QPushButton("清除PID")
        self.flash_erase_ff_btn = QPushButton("清除前馈")
        self.flash_erase_map_btn = QPushButton("清除地图")
        self.flash_erase_btn = QPushButton("清除全部")
        btn_row.addWidget(self.param_refresh_btn)
        btn_row.addWidget(self.param_apply_btn)
        btn_row.addWidget(self.flash_enable_btn)
        btn_row.addWidget(self.params_query_btn)
        btn_row.addWidget(self.flash_query_btn)
        btn_row.addWidget(self.flash_save_pid_btn)
        btn_row.addWidget(self.flash_save_ff_btn)
        btn_row.addWidget(self.flash_save_map_btn)
        btn_row.addWidget(self.flash_save_btn)
        btn_row.addWidget(self.flash_load_btn)
        btn_row.addWidget(self.flash_erase_pid_btn)
        btn_row.addWidget(self.flash_erase_ff_btn)
        btn_row.addWidget(self.flash_erase_map_btn)
        btn_row.addWidget(self.flash_erase_btn)
        btn_row.addStretch(1)

        self.param_tabs = QTabWidget()
        self.param_tables: list[QTableWidget] = []
        categories: dict[str, list] = {}
        for param in self.device_schema.ground_params:
            categories.setdefault(param.category, []).append(param)
        preferred = ["运动控制", "电机", "控制算法", "传感器", "地图/策略", "通信/遥测", "显示", "通用"]
        ordered_categories = [c for c in preferred if c in categories] + [c for c in categories if c not in preferred]
        for category in ordered_categories:
            params = categories[category]
            table = QTableWidget(len(params), 4)
            table.setHorizontalHeaderLabels(["名称", "值/开关", "命令", "说明"])
            table.setSelectionBehavior(QAbstractItemView.SelectRows)
            table.setSelectionMode(QAbstractItemView.SingleSelection)
            for row, param in enumerate(params):
                self.param_row_by_name[param.name] = (table, row)
                table.setItem(row, 0, QTableWidgetItem(param.name))
                value_item = QTableWidgetItem("" if param.type == "pid" else param.default)
                if param.type == "pid":
                    value_item.setToolTip(param.default)
                table.setItem(row, 1, value_item)
                if param.type == "bool":
                    cb = QCheckBox("开")
                    cb.setChecked(str(param.default).upper() in ("1", "ON", "TRUE", "YES"))
                    table.setCellWidget(row, 1, cb)
                elif param.type == "pid":
                    table.setCellWidget(row, 1, self.create_pid_value_widget(param.name, param.default))
                table.setItem(row, 2, QTableWidgetItem(param.command))
                desc = param.description
                if param.danger:
                    desc = f"[需二次确认] {desc}"
                table.setItem(row, 3, QTableWidgetItem(desc))
            table.resizeColumnsToContents()
            self.param_tables.append(table)
            self.param_tabs.addTab(table, category)

        layout.addWidget(hint)
        layout.addLayout(btn_row)
        layout.addWidget(self.param_tabs)
        return tab

    def create_pid_value_widget(self, param_name: str, value: str) -> QWidget:
        widget = QWidget()
        widget.setProperty("pid_editor", True)
        widget.setProperty("pid_param_name", param_name)
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        parts = value.split()
        defaults = [0.0, 0.0, 0.0]
        for i, part in enumerate(parts[:3]):
            try:
                defaults[i] = float(part)
            except ValueError:
                pass
        for name, default in zip(("Kp", "Ki", "Kd"), defaults):
            label = QLabel(name)
            label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            label.setMinimumWidth(22)
            spin = QDoubleSpinBox()
            spin.setObjectName(name.lower())
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(4)
            spin.setSingleStep(1.0 if name != "Kd" else 0.1)
            spin.setValue(default)
            spin.setAlignment(Qt.AlignCenter)
            spin.setMinimumWidth(92)
            spin.valueChanged.connect(lambda _value, n=param_name: self.dirty_pid_params.add(n))
            layout.addWidget(label)
            layout.addWidget(spin)
        layout.addStretch(1)
        return widget

    def read_pid_value_widget(self, widget: QWidget) -> str:
        values = []
        for name in ("kp", "ki", "kd"):
            spin = widget.findChild(QDoubleSpinBox, name)
            values.append(spin.value() if spin else 0.0)
        return " ".join(f"{v:.4g}" for v in values)

    def normalize_pid_value(self, value: str) -> str:
        parts = value.split()
        values = []
        for part in parts[:3]:
            try:
                values.append(float(part))
            except ValueError:
                values.append(0.0)
        while len(values) < 3:
            values.append(0.0)
        return " ".join(f"{v:.4g}" for v in values)

    def set_pid_value_widget(self, widget: QWidget, value: str) -> None:
        parts = value.split()
        for name, part in zip(("kp", "ki", "kd"), parts[:3]):
            spin = widget.findChild(QDoubleSpinBox, name)
            if not spin:
                continue
            try:
                spin.blockSignals(True)
                spin.setValue(float(part))
            except ValueError:
                pass
            finally:
                spin.blockSignals(False)

    def pid_value_widget_has_focus(self, widget: QWidget) -> bool:
        for name in ("kp", "ki", "kd"):
            spin = widget.findChild(QDoubleSpinBox, name)
            if spin and spin.hasFocus():
                return True
        return False

    def pid_param_name_for_channel(self, channel_id: int) -> str:
        if channel_id == 0:
            return "循迹PID"
        if channel_id == 1:
            return "左轮速度PID"
        if channel_id == 2:
            return "右轮速度PID"
        if channel_id == 254:
            return "双轮速度PID"
        return ""

    def channel_for_pid_param_name(self, name: str) -> int | None:
        return {
            "循迹PID": 0,
            "左轮速度PID": 1,
            "右轮速度PID": 2,
            "双轮速度PID": 254,
        }.get(name)

    def set_pid_param_value(self, name: str, value: str, force: bool = False) -> None:
        row_info = self.param_row_by_name.get(name)
        if row_info is None:
            return
        table, row = row_info
        widget = table.cellWidget(row, 1)
        if isinstance(widget, QWidget) and widget.property("pid_editor"):
            if not force and name in self.dirty_pid_params:
                return
            if not force and self.pid_value_widget_has_focus(widget):
                return
            self.set_pid_value_widget(widget, value)
            if force:
                self.dirty_pid_params.discard(name)
        item = table.item(row, 1)
        if item:
            item.setText("")
            item.setToolTip(value)

    def sync_pid_from_channel(self, channel_id: int, kp: float, ki: float, kd: float) -> None:
        value = f"{kp:.4g} {ki:.4g} {kd:.4g}"
        pending = self.pending_pid_values_by_channel.get(channel_id)
        if pending is not None:
            if value != pending:
                return
            self.pending_pid_values_by_channel.pop(channel_id, None)
        name = self.pid_param_name_for_channel(channel_id)
        if name:
            self.set_pid_param_value(name, value)
        if channel_id in (1, 2):
            # Keep the shared row meaningful when both wheel gains are currently identical.
            other_name = "右轮速度PID" if channel_id == 1 else "左轮速度PID"
            other_info = self.param_row_by_name.get(other_name)
            if other_info is not None:
                other_widget = other_info[0].cellWidget(other_info[1], 1)
                if isinstance(other_widget, QWidget) and other_widget.property("pid_editor"):
                    if self.read_pid_value_widget(other_widget) == value:
                        self.set_pid_param_value("双轮速度PID", value)

    def sync_pid_command_to_ui(self, target: str, value: str) -> None:
        target = target.upper()
        value = self.normalize_pid_value(value)
        if target == "LINE":
            names = ["循迹PID"]
            channels = [0]
        elif target == "LEFT":
            names = ["左轮速度PID"]
            channels = [1]
        elif target == "RIGHT":
            names = ["右轮速度PID"]
            channels = [2]
        elif target in ("SPEED", "BOTH"):
            names = ["左轮速度PID", "右轮速度PID", "双轮速度PID"]
            channels = [1, 2, 254]
        else:
            return
        for name in names:
            self.set_pid_param_value(name, value, force=True)
        for channel in channels:
            self.pending_pid_values_by_channel[channel] = value

    def _bind_events(self) -> None:
        self.thread.started.connect(self.start_worker_connection)
        self.connect_requested.connect(self.worker.connect_port)
        
        self.worker.status.connect(self.status.setText)
        self.worker.text_received.connect(self.append_console_line)
        self.btn.clicked.connect(self.toggle_connect)
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        self.plot_tabs.currentChanged.connect(self.on_plot_tab_changed)
        self.advanced_btn.clicked.connect(self.advanced_win.show)
        self.device_config_btn.clicked.connect(lambda: self.show_tool_window(self.device_config_win))
        self.console_win_btn.clicked.connect(lambda: self.show_tool_window(self.console_win))
        self.wave_win_btn.clicked.connect(lambda: self.show_tool_window(self.wave_win))
        self.map_win_btn.clicked.connect(lambda: self.show_tool_window(self.map_win))
        self.save_csv_btn.clicked.connect(self.save_current_channel_csv)
        self.save_all_csv_btn.clicked.connect(self.save_all_channels_csv)
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
        self.straight_test_btn.clicked.connect(self.start_straight_line_test)
        self.race_track_btn.clicked.connect(self.start_race_tracking)
        self.spin_test_btn.clicked.connect(self.toggle_spin_test)
        self.speed_tune_btn.clicked.connect(self.toggle_speed_pid_tune)
        self.step_sample_btn.clicked.connect(self.start_semi_auto_step_sample)
        self.spin_stop_btn.clicked.connect(self.stop_spin_test)
        self.analyze_step_btn.clicked.connect(self.run_step_analysis)
        self.apply_model_btn.clicked.connect(self.apply_latest_model_recommendation)
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
        self.flash_enable_btn.clicked.connect(self.enable_flash_storage)
        self.params_query_btn.clicked.connect(self.query_current_params)
        self.flash_query_btn.clicked.connect(self.query_flash_storage)
        self.flash_save_pid_btn.clicked.connect(lambda: self.save_flash_storage("PID"))
        self.flash_save_ff_btn.clicked.connect(lambda: self.save_flash_storage("FF"))
        self.flash_save_map_btn.clicked.connect(lambda: self.save_flash_storage("MAP"))
        self.flash_save_btn.clicked.connect(self.save_flash_storage)
        self.flash_load_btn.clicked.connect(self.load_flash_storage)
        self.flash_erase_pid_btn.clicked.connect(lambda: self.erase_flash_storage("PID"))
        self.flash_erase_ff_btn.clicked.connect(lambda: self.erase_flash_storage("FF"))
        self.flash_erase_map_btn.clicked.connect(lambda: self.erase_flash_storage("MAP"))
        self.flash_erase_btn.clicked.connect(self.erase_flash_storage)
        for table in self.param_tables:
            table.itemSelectionChanged.connect(self._on_param_table_selection_changed)

    def toggle_connect(self):
        if self.thread.isRunning():
            self.stop_worker_thread()
            self.btn.setText("连接")
            self.status.setText("已断开")
        else:
            baud = int(self.baud.currentText())
            self.btn.setText("断开")
            self.thread.start()
            self.btn.setText("断开")
            if baud <= 9600:
                self.status.setText(f"已连接 @{baud} — 警告：低波特率下二进制遥测+文本命令共用串口，请勿快速连续发送命令，否则可能导致数据中断。建议 HC-05 AT+UART 改为 115200 以上。")
            else:
                self.status.setText("正在连接...")

    def stop_worker_thread(self):
        self.worker.stop()
        self.thread.quit()
        if not self.thread.wait(1500):
            self.thread.terminate()
            self.thread.wait(1000)

    def closeEvent(self, event):
        self.timer.stop()
        for win in (self.advanced_win, self.device_config_win, self.console_win, self.wave_win, self.map_win):
            win.close()
        if self.thread.isRunning():
            self.stop_worker_thread()
        event.accept()

    def show_tool_window(self, window: QWidget):
        window.show()
        window.raise_()
        window.activateWindow()

    def describe_status_flags(self, flags: int) -> list[str]:
        if self.device_schema.status_flags:
            items = [name for bit, name in sorted(self.device_schema.status_flags.items()) if flags & (1 << bit)]
            return items or ["空闲"]
        return describe_status_flags(flags)

    def scene_status_text(self) -> str:
        status = self.latest_map_status
        if not status:
            return "场景：等待MAP_STATUS"
        current_type = int(status.get("current_type", 0))
        next_type = int(status.get("next_type", 0))
        car_mode = int(status.get("car_mode", 0))
        current_id = int(status.get("current_id", 0xFFFF))
        next_id = int(status.get("next_id", 0xFFFF))
        current_name = SEGMENT_NAMES.get(current_type, "UNKNOWN")
        next_name = SEGMENT_NAMES.get(next_type, "UNKNOWN")
        mode_name = CAR_MODE_NAMES.get(car_mode, "UNKNOWN")
        current_text = "无" if current_id == 0xFFFF else f"{current_id}:{current_name}"
        next_text = "无" if next_id == 0xFFFF else f"{next_id}:{next_name}"
        return f"场景：模式={mode_name} 当前={current_text} 下一={next_text} 距下一={float(status.get('dist_to_next_m', -1.0)):.2f}m"

    def control_status_text(self, flags: int | None = None) -> str:
        if flags is None:
            flags = self.latest_status_flags
        return "控制：" + "/".join(self.describe_status_flags(int(flags)))

    def combined_status_text(self, flags: int | None = None) -> str:
        return f"{self.scene_status_text()}\n{self.control_status_text(flags)}"

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
        plot.setLabel("left", "目标/反馈/误差", units="m/s")
        plot.showAxis("right")
        plot.getAxis("right").setLabel("输出", units="PWM")
        plot.setTitle(f"{key[0]}:{key[1]} {name}")
        plot.setClipToView(True)
        plot.setDownsampling(auto=True, mode="peak")
        plot.enableAutoRange(axis="y", enable=True)
        legend = plot.addLegend(offset=(10, 10))
        plot.addLine(y=0, pen=pg.mkPen("#495057", width=1, style=Qt.DashLine))
        output_view = pg.ViewBox()
        plot.scene().addItem(output_view)
        plot.getAxis("right").linkToView(output_view)
        output_view.setXLink(plot)
        output_view.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

        def update_output_view():
            output_view.setGeometry(plot.getViewBox().sceneBoundingRect())
            output_view.linkedViewChanged(plot.getViewBox(), output_view.XAxis)

        plot.getViewBox().sigResized.connect(update_output_view)
        curves = {
            k: plot.plot(pen=pg.mkPen(c, width=width, style=style), name=label)
            for k, c, width, style, label in [
                ("feedback", "#22b8cf", 2, Qt.SolidLine, "左实际" if key[1] == 254 else "反馈"),
                ("error", "#da77f2", 1, Qt.SolidLine, "右实际" if key[1] == 254 else "误差"),
                ("target", "#ffd43b", 3, Qt.DashLine, "目标"),
            ]
        }
        curves["output"] = pg.PlotDataItem(pen=pg.mkPen("#f8f9fa", width=1, style=Qt.SolidLine), name="输出")
        output_view.addItem(curves["output"])
        legend.addItem(curves["output"], "输出")
        update_output_view()
        self.plots[key] = plot
        self.curves_by_channel[key] = curves
        if "output" in curves:
            curves["output"].setVisible(False)
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
        stripped = cmd.strip()
        parts = stripped.split()
        if parts:
            cmd_key = " ".join(parts[:3]) if len(parts) >= 3 else parts[0]
            if cmd_key not in ("STOP", "$STOP"):
                now = time.monotonic()
                last = self._last_cmd_send_s.get(cmd_key, 0.0)
                if now - last < self._cmd_send_min_interval_s:
                    return
                self._last_cmd_send_s[cmd_key] = now
        self.remote_command_requested.emit(cmd)
        self.apply_command_to_param_state(cmd)
        self.status.setText(f"已发送远控命令：{stripped}")

    def send_remote_commands(self, commands: list[str]):
        for cmd in commands:
            self.send_remote_command(cmd if cmd.endswith("\n") else cmd + "\n")
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
            speed = 0.225
            self.remote_speed.setValue(speed)
            self.set_param_value("目标速度", f"{speed:.3f}")
        return speed

    def set_param_value(self, name: str, value: str):
        row_info = self.param_row_by_name.get(name)
        if row_info is None:
            return
        table, row = row_info
        widget = table.cellWidget(row, 1)
        if isinstance(widget, QCheckBox):
            widget.blockSignals(True)
            widget.setChecked(str(value).upper() in ("1", "ON", "TRUE", "YES"))
            widget.blockSignals(False)
            item = table.item(row, 1)
            if item:
                item.setText("1" if widget.isChecked() else "0")
        elif isinstance(widget, QWidget) and widget.property("pid_editor"):
            self.set_pid_param_value(name, value, force=True)
        else:
            item = table.item(row, 1)
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

        flash_on = self.runtime_state.get("FLASH_STORAGE") == "1"
        self.flash_enable_btn.setText("Flash已启用" if flash_on else "启用Flash功能")

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
        elif parts[0] == "PID" and len(parts) >= 5:
            self.sync_pid_command_to_ui(parts[1], " ".join(parts[2:5]))
        elif parts[0] in ("FF", "FEEDFORWARD") and len(parts) >= 4:
            target = parts[1].upper()
            value = " ".join(parts[2:4])
            names = []
            if target == "LEFT":
                names = ["左轮前馈模型"]
            elif target == "RIGHT":
                names = ["右轮前馈模型"]
            elif target in ("BOTH", "SPEED"):
                names = ["左轮前馈模型", "右轮前馈模型", "双轮前馈模型"]
            for name in names:
                self.set_param_value(name, value)

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

    def start_straight_line_test(self):
        speed = self.get_test_speed()
        result = QMessageBox.warning(
            self,
            "确认长直线测试",
            f"车辆会按 TRACKING 模式真实输出电机，目标速度 {speed:.3f}m/s。请确认车辆在长直线赛道上且前方安全。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if result == QMessageBox.Yes:
            self.apply_command_plan(self.command_router.start_straight_line_test(speed))

    def start_race_tracking(self):
        result = QMessageBox.warning(
            self,
            "确认启动循迹比赛",
            "将关闭上位机接管并启动真实自主循迹代码，车辆会按下位机策略运行。请确认车辆在赛道起点且前方安全。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if result == QMessageBox.Yes:
            self.apply_command_plan(self.command_router.start_race_tracking())

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
            self.select_channel(1, 254)
        self.clear_current_plot()
        QTimer.singleShot(800, lambda s=speed, t=target: self.fire_speed_pid_tune(s, t))
        self.status.setText(f"悬空闭环调参准备中：目标={self.tune_target.currentText()}，先保持 0 目标 800ms 再启动。")

    def fire_speed_pid_tune(self, speed: float, target: str):
        if self.runtime_state.get("SPEED_TUNE") != "1":
            return
        self.apply_command_plan(self.command_router.fire_speed_pid_tune(speed, target))

    def start_semi_auto_step_sample(self):
        target = self.tune_target.currentData() or "LEFT"
        if target == "BOTH":
            target = "LEFT"
            self.tune_target.setCurrentIndex(max(0, self.tune_target.findData("LEFT")))
        channel_id = 2 if target == "RIGHT" else 1
        self.select_channel(1, channel_id)
        self.clear_current_plot()
        self.apply_command_plan(self.command_router.prepare_step_sample(target))
        QTimer.singleShot(800, lambda t=target: self.fire_semi_auto_step(t))

    def fire_semi_auto_step(self, target: str):
        if self.runtime_state.get("SPEED_TUNE") != "1":
            return
        speed = self.get_test_speed()
        self.apply_command_plan(self.command_router.fire_step_sample(target, speed))

    def start_motor_test(self, mode: str):
        pwm = self.motor_test_pwm.value()
        self.apply_command_plan(self.command_router.start_motor_test(mode, pwm))

    def stop_motor_test(self):
        self.apply_command_plan(self.command_router.stop_motor_test())

    def stop_spin_test(self):
        self.apply_command_plan(self.command_router.stop_spin_test())

    def enable_flash_storage(self):
        self.apply_command_plan(self.command_router.enable_flash_storage())

    def query_current_params(self):
        self.apply_command_plan(self.command_router.query_current_params())

    def query_flash_storage(self):
        self.apply_command_plan(self.command_router.query_flash_storage())

    def save_flash_storage(self, section: str = "ALL"):
        self.apply_command_plan(self.command_router.save_flash_storage(section))

    def load_flash_storage(self):
        self.apply_command_plan(self.command_router.load_flash_storage())

    def erase_flash_storage(self, section: str = "ALL"):
        label = {"PID": "PID参数", "FF": "前馈模型", "MAP": "辅助地图", "PARAMS": "PID和前馈", "ALL": "全部保存区"}.get(section, section)
        result = QMessageBox.warning(
            self,
            "确认擦除Flash",
            f"将擦除下位机 Flash 保存区中的 {label}。此操作不可撤销。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if result == QMessageBox.Yes:
            self.apply_command_plan(self.command_router.erase_flash_storage(section))

    def clear_current_plot(self):
        d, c = self.current_channel
        if c == 254:
            self.dm.clear_channel(d, 1)
            self.dm.clear_channel(d, 2)
        else:
            self.dm.clear_channel(d, c)
        self.cycle_counts[(d, c)] = 0
        for curve in self.curves_by_channel.get((d, c), {}).values():
            curve.setData([])
        self.analysis.clear()
        self.channel_io.setText("目标/实际速度：已清除")
        self.channel_error.setText("误差/输出PWM：已清除")
        self.channel_gain.setText("PID参数：已清除")
        self.channel_state.setText("状态：已清除当前通道波形")

    def clear_all_plots(self):
        self.dm.clear_all()
        self.cycle_counts.clear()
        for curves in self.curves_by_channel.values():
            for curve in curves.values():
                curve.setData([])
        self.analysis.clear()
        self.channel_io.setText("目标/实际速度：已清除")
        self.channel_error.setText("误差/输出PWM：已清除")
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
        if line.startswith("$OK,STORAGE,") or line.startswith("$ERR,STORAGE,"):
            self.status.setText(line)
            return
        if line.startswith("$PARAMS,") or line.startswith("$STORAGE,"):
            self.status.setText(line)
            self.apply_param_snapshot_reply(line)
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

    def apply_param_snapshot_reply(self, line: str):
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            return
        if parts[0] == "$STORAGE" and len(parts) >= 2 and parts[1] == "EMPTY":
            self.status.setText("Flash 保存区为空或校验无效。")
            return
        label = "当前RAM" if parts[0] == "$PARAMS" else "Flash保存"
        sections = 7 if parts[0] == "$PARAMS" else 0
        if "SEC" in parts:
            try:
                sec_index = parts.index("SEC")
                sections = int(parts[sec_index + 1])
            except (ValueError, IndexError):
                sections = 0
        i = 1
        summary = []
        while i < len(parts):
            key = parts[i]
            if key in ("LINE", "LEFT", "RIGHT") and i + 3 < len(parts):
                value = " ".join(parts[i + 1:i + 4])
                if (sections & 1) == 0:
                    summary.append(f"{key} PID=<未保存>")
                elif key == "LINE":
                    self.set_pid_param_value("循迹PID", value, force=True)
                    summary.append(f"{key} PID={value}")
                elif key == "LEFT":
                    self.set_pid_param_value("左轮速度PID", value, force=True)
                    summary.append(f"{key} PID={value}")
                elif key == "RIGHT":
                    self.set_pid_param_value("右轮速度PID", value, force=True)
                    summary.append(f"{key} PID={value}")
                i += 4
            elif key in ("FFL", "FFR") and i + 2 < len(parts):
                value = " ".join(parts[i + 1:i + 3])
                if (sections & 2) == 0:
                    summary.append(f"{key}=<未保存>")
                else:
                    self.set_param_value("左轮前馈模型" if key == "FFL" else "右轮前馈模型", value)
                    summary.append(f"{key}={value}")
                i += 3
            elif key == "MAP" and i + 1 < len(parts):
                summary.append(f"MAP={parts[i + 1] if (sections & 4) else '<未保存>'}")
                i += 2
            elif key == "SEC" and i + 1 < len(parts):
                summary.append(f"SEC={parts[i + 1]}")
                i += 2
            else:
                i += 1
        if summary:
            self.analysis.setText(label + "参数：\n" + "\n".join(summary))
            self.status.setText(f"已读取{label}参数。")

    def apply_selected_param(self):
        table = self.param_tabs.currentWidget()
        if not isinstance(table, QTableWidget):
            return
        row = table.currentRow()
        if row < 0:
            return
        name_item = table.item(row, 0)
        value_item = table.item(row, 1)
        command_item = table.item(row, 2)
        if not name_item or not value_item or not command_item:
            return
        name = name_item.text().strip()
        spec = self.param_specs_by_name.get(name)
        widget = table.cellWidget(row, 1)
        if isinstance(widget, QCheckBox):
            value = "1" if widget.isChecked() else "0"
            value_item.setText(value)
        elif isinstance(widget, QWidget) and widget.property("pid_editor"):
            value = self.read_pid_value_widget(widget)
            value_item.setText("")
            value_item.setToolTip(value)
        else:
            value = value_item.text().strip()
        command = command_item.text().strip()
        if "{value}" not in command:
            self.send_remote_command(command + "\n")
            return
        if not value or value.lower() == "read only":
            self.status.setText("选中的参数是只读项")
            return
        final_command = command.replace("{value}", value)
        danger_on = spec and spec.danger and value.upper() not in ("0", "OFF", "FALSE", "NO")
        if danger_on:
            confirm_key = spec.key
            if self.pending_safety_feature != confirm_key:
                self.pending_safety_feature = confirm_key
                self.highlight_param_row(table, row, True)
                self._safety_confirm_timer.start(5000)
                arm_command = spec.arm_command.strip() if spec.arm_command else ""
                if not arm_command:
                    parts = final_command.split()
                    if len(parts) >= 4 and parts[:2] == ["CFG", "SET"]:
                        arm_command = f"CFG ARM {parts[2]}"
                if arm_command:
                    self.send_remote_command(arm_command + "\n")
                self.status.setText(f'{name} 已预确认并高亮，请在5秒内再次点击"应用选中项"真正打开。')
                return
            self._safety_confirm_timer.stop()
            self.pending_safety_feature = ""
        self.highlight_param_row(table, row, False)
        self.send_remote_command(final_command + "\n")
        if spec and spec.type == "pid":
            parts = final_command.split()
            if len(parts) >= 5 and parts[0] == "PID":
                self.sync_pid_command_to_ui(parts[1], value)

    def _clear_safety_highlight(self):
        self.pending_safety_feature = ""
        if self._highlighted_row is not None:
            table, row = self._highlighted_row
            self._clear_row_background(table, row)
            self._highlighted_row = None
        self.status.setText("安全确认超时，高亮已自动清除。请重新选择参数。")

    def _on_param_table_selection_changed(self):
        if self._highlighted_row is None:
            return
        table = self.sender()
        if not isinstance(table, QTableWidget):
            return
        row = table.currentRow()
        if row < 0:
            return
        highlighted_table, highlighted_row = self._highlighted_row
        if table is not highlighted_table or row != highlighted_row:
            self._safety_confirm_timer.stop()
            self._clear_safety_highlight()

    def highlight_param_row(self, table: QTableWidget, row: int, enabled: bool):
        if self._highlighted_row is not None:
            old_table, old_row = self._highlighted_row
            if old_table is not table or old_row != row:
                self._clear_row_background(old_table, old_row)
        if enabled:
            self._set_row_background(table, row, QColor("#fff3bf"))
            self._highlighted_row = (table, row)
        else:
            self._clear_row_background(table, row)
            self._highlighted_row = None

    def _set_row_background(self, table: QTableWidget, row: int, color: QColor):
        for col in range(table.columnCount()):
            item = table.item(row, col)
            if item:
                item.setBackground(color)

    def _clear_row_background(self, table: QTableWidget, row: int):
        for col in range(table.columnCount()):
            item = table.item(row, col)
            if item:
                item.setData(Qt.BackgroundRole, None)

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

    def save_all_channels_csv(self):
        folder = QFileDialog.getExistingDirectory(self, "选择保存全部通道CSV的文件夹")
        if not folder:
            return
        saved = 0
        total_rows = 0
        for d, c in sorted(self.dm.channels()):
            path = Path(folder) / f"pidscope_device{d}_ch{c}.csv"
            count = self.dm.export_channel_csv(d, c, str(path))
            if count > 0:
                saved += 1
                total_rows += count
        if saved == 0:
            QMessageBox.information(self, "保存CSV", "当前没有可保存的遥测数据。")
            return
        self.status.setText(f"已保存 {saved} 个通道、{total_rows} 行遥测数据到 {folder}")
        QMessageBox.information(self, "保存CSV", f"已保存 {saved} 个通道，共 {total_rows} 行。")

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
        return self.channel_defs.get((device_id, channel_id), f"通道{channel_id}")

    def on_frame_ready(self, frame: PIDFrame, update_advanced: bool = True):
        if frame.frame_type == FRAME_HEARTBEAT:
            try:
                hb = decode_heartbeat(frame.payload)
                self.latest_heartbeat_ms = int(hb["uptime_ms"])
                self.latest_status_flags = int(hb["status_flags"])
                self.status.setText(f"心跳 {self.latest_heartbeat_ms} ms | {self.scene_status_text()} | {self.control_status_text()}")
            except Exception:
                pass
            return None
        if frame.frame_type == FRAME_MAP_STATUS:
            try:
                map_status = decode_map_status(frame.payload)
                self.latest_status_flags = int(map_status.get("status_flags", self.latest_status_flags))
                self.latest_map_status = map_status
                self.status.setText(f"心跳 {self.latest_heartbeat_ms} ms | {self.scene_status_text()} | {self.control_status_text()}")
                if self.map_win.isVisible():
                    self.map_win.update_map_status(map_status, self.control_status_text())
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
                self.map_win.update_from_telemetry(tel, self.combined_status_text())
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
            self.map_win.update_from_telemetry(tel, self.combined_status_text())
        if self.worker.dropped_frames:
            self.status.setText(f"遥测过载：已丢弃 {self.worker.dropped_frames} 帧，UI 仍按固定帧率刷新")
            self.worker.dropped_frames = 0
        return len(frames)

    def refresh_plot(self):
        self.drain_serial_frames()
        self.refresh_channel_plot(*self.current_channel)

    def refresh_channel_plot(self, d: int, c: int):
        if c == 254:
            self.refresh_both_speed_plot(d)
            return
        data = self.dm.get_latest_arrays(d, c, count=int(self.plot_samples.value()))
        sample_count = len(data["timestamp_ms"])
        if sample_count < 2:
            if (d, c) == self.current_channel:
                scope_on = self.runtime_state.get("PID_SCOPE") == "1" and self.runtime_state.get("PID_SCOPE_OVER_HC05") == "1"
                hint = "等待更多样本" if scope_on else "波形流已关闭，点击开启波形或启动调参"
                self.channel_state.setText(f"状态：{hint}，当前 {sample_count} 个\n{self.combined_status_text()}")
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
        self.sync_pid_from_channel(c, float(last["kp"]), float(last["ki"]), float(last["kd"]))
        self.channel_io.setText(f"目标速度：{last['target']:.3f} m/s\n实际速度：{last['feedback']:.3f} m/s")
        self.channel_error.setText(f"速度误差：{last['error']:.3f} m/s\n输出PWM：{last['output']:.1f}")
        self.channel_gain.setText(f"Kp：{last['kp']:.3f}\nKi：{last['ki']:.3f}\nKd：{last['kd']:.3f}")
        total_rows = len(self.dm.get_rows(d, c))
        self.channel_state.setText(
            f"窗口样本：{sample_count}\n累计样本：{total_rows}\n"
            f"时间：{int(last['timestamp_ms'])} ms\n"
            f"{self.combined_status_text(int(last.get('status_flags', self.latest_status_flags)))}"
        )
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
        self.handle_auto_cycle(d, c)

    def refresh_both_speed_plot(self, d: int):
        left = self.dm.get_latest_arrays(d, 1, count=int(self.plot_samples.value()))
        right = self.dm.get_latest_arrays(d, 2, count=int(self.plot_samples.value()))
        left_count = len(left["timestamp_ms"])
        right_count = len(right["timestamp_ms"])
        if left_count < 2 and right_count < 2:
            scope_on = self.runtime_state.get("PID_SCOPE") == "1" and self.runtime_state.get("PID_SCOPE_OVER_HC05") == "1"
            hint = "等待左右轮样本" if scope_on else "波形流已关闭，点击开启波形或启动调参"
            self.channel_state.setText(f"状态：{hint}，左 {left_count} 个，右 {right_count} 个\n{self.combined_status_text()}")
            return
        curves = self.curves_by_channel.get((d, 254), {})
        first_ts = None
        for data in (left, right):
            if len(data["timestamp_ms"]):
                ts = float(data["timestamp_ms"][0])
                first_ts = ts if first_ts is None else min(first_ts, ts)
        first_ts = first_ts or 0.0
        if left_count >= 2:
            lx = left["timestamp_ms"] / 1000.0 - first_ts / 1000.0
            if "target" in curves:
                curves["target"].setData(lx, left["target"])
            if "feedback" in curves:
                curves["feedback"].setData(lx, left["feedback"])
            if "output" in curves:
                curves["output"].setData(lx, left["output"])
        if right_count >= 2:
            rx = right["timestamp_ms"] / 1000.0 - first_ts / 1000.0
            if "error" in curves:
                curves["error"].setData(rx, right["feedback"])
        last = left if left_count else right
        last_row = {name: values[-1] for name, values in last.items()}
        self.sync_pid_from_channel(254, float(last_row["kp"]), float(last_row["ki"]), float(last_row["kd"]))
        left_last = float(left["feedback"][-1]) if left_count else 0.0
        right_last = float(right["feedback"][-1]) if right_count else 0.0
        target_last = float(last_row["target"])
        self.channel_io.setText(f"目标速度：{target_last:.3f} m/s\n左实际：{left_last:.3f} m/s\n右实际：{right_last:.3f} m/s")
        self.channel_error.setText(f"左误差：{(target_last - left_last):.3f} m/s\n右误差：{(target_last - right_last):.3f} m/s")
        self.channel_gain.setText(f"Kp：{last_row['kp']:.3f}\nKi：{last_row['ki']:.3f}\nKd：{last_row['kd']:.3f}")
        self.channel_state.setText(
            f"窗口样本：左 {left_count} / 右 {right_count}\n"
            f"累计样本：左 {len(self.dm.get_rows(d, 1))} / 右 {len(self.dm.get_rows(d, 2))}\n"
            f"时间：{int(last_row['timestamp_ms'])} ms\n"
            f"{self.combined_status_text(int(last_row.get('status_flags', self.latest_status_flags)))}"
        )

    def handle_auto_cycle(self, d: int, c: int):
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
        if c == 254:
            self.analysis.setText("双轮速度页用于观察左右轮同步，不做单通道阶跃分析；请切到左轮或右轮速度PID页分析。")
            return
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
        model_lines = self.motor_model_advice(window)
        if model_lines:
            lines.append("")
            lines.extend(model_lines)
        self.analysis.setText("\n".join(lines))

    def motor_model_advice(self, window: list[dict]) -> list[str]:
        self.latest_model_recommendation = None
        self.apply_model_btn.setEnabled(False)
        if self.current_channel[1] not in (1, 2):
            return []
        model = identify_motor_step_model(window)
        status = model.get("status")
        if status != "ok":
            labels = {
                "not_enough_data": "数据不足，无法做模型辨识。",
                "no_step": "没有检测到速度阶跃，无法做模型辨识。",
                "zero_target": "目标速度为 0，无法估算速度模型。",
                "weak_response": "反馈变化太小，无法可靠辨识电机模型。",
                "bad_gain": "估算过程增益异常，无法给出 IMC-PI。",
            }
            return ["模型辨识/IMC-PI：" + labels.get(str(status), f"状态={status}")]
        quality = model.get("quality") or []
        channel_name = self.channel_name(*self.current_channel)
        target_name = "LEFT" if self.current_channel[1] == 1 else "RIGHT"
        pwm_per_mps = float(model["recommended_ff_pwm"]) / max(abs(float(model["target"])), 1e-6)
        self.latest_model_recommendation = {
            "target": target_name,
            "deadband_pwm": 0.0,
            "pwm_per_mps": pwm_per_mps,
            "kp": float(model["recommended_kp"]),
            "ki": float(model["recommended_ki"]),
        }
        self.apply_model_btn.setEnabled(True)
        lines = [
            f"模型辨识/IMC-PI：{channel_name}",
            f"稳态反馈≈{model['steady_feedback']:.3f}m/s，峰值≈{model['peak_feedback']:.3f}m/s，稳态误差≈{model['steady_error']:.3f}m/s",
            f"估算前馈≈{model['feedforward_pwm']:.0f}PWM，建议前馈≈{model['recommended_ff_pwm']:.0f}PWM，建议 SPEED_FEEDFORWARD_GAIN≈{model['recommended_ff_gain']:.3f}",
            f"模型：K≈{model['plant_gain_mps_per_pwm']:.6f} m/s/PWM，T≈{model['tau_s']:.2f}s，L≈{model['delay_s']:.2f}s，lambda≈{model['lambda_s']:.2f}s",
            f"IMC原始PI：Kp={model['raw_imc_kp']:.4g}, Ki={model['raw_imc_ki']:.4g}, Kd=0",
            f"分阶段安全建议：Kp={model['recommended_kp']:.4g}, Ki={model['recommended_ki']:.4g}, Kd=0",
            f"可下发命令：FF {target_name} 0 {pwm_per_mps:.1f}；PID {target_name} {model['recommended_kp']:.4g} {model['recommended_ki']:.4g} 0",
        ]
        if quality:
            lines.append("数据质量提示：" + "；".join(quality))
        lines.append("说明：IMC-PI 是基于当前阶跃窗口的保守模型建议；落地负载变化后需重新辨识。")
        return lines

    def apply_latest_model_recommendation(self):
        rec = self.latest_model_recommendation
        if not rec:
            self.status.setText("没有可应用的模型建议；请先在左/右速度通道执行阶跃分析。")
            return
        self.apply_command_plan(self.command_router.apply_model_recommendation(
            rec["target"], rec["deadband_pwm"], rec["pwm_per_mps"], rec["kp"], rec["ki"]
        ))

    def pid_tuning_advice(self, window: list[dict], result: dict) -> list[str]:
        if result.get("status") == "no_step":
            return ["PID建议：当前窗口没有检测到目标阶跃，不能计算超调或给调参建议；请先清空波形，再执行半自动阶跃采样。"]
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
        last_error = float(last.get("error", 0.0))
        is_speed_channel = self.current_channel[1] in (1, 2, 254)

        targets = [float(row.get("target", 0.0)) for row in window]
        feedbacks = [float(row.get("feedback", 0.0)) for row in window]
        errors = [float(row.get("error", t - f)) for row, t, f in zip(window, targets, feedbacks)]
        outputs = [float(row.get("output", 0.0)) for row in window]
        p_terms = [float(row.get("p_term", 0.0)) for row in window]
        i_terms = [float(row.get("i_term", 0.0)) for row in window]
        d_terms = [float(row.get("d_term", 0.0)) for row in window]
        tail_count = max(5, len(window) // 5)
        tail_feedback = sum(feedbacks[-tail_count:]) / tail_count
        tail_error = sum(errors[-tail_count:]) / tail_count
        tail_output = sum(outputs[-tail_count:]) / tail_count
        peak_feedback = max(feedbacks)
        final_target = targets[-1]
        tolerance = max(0.01, abs(final_target) * 0.05)
        meaningful_deadband = max(0.006, abs(final_target) * 0.025)
        meaningful_signs: list[int] = []
        for err in errors:
            if abs(err) <= meaningful_deadband:
                continue
            sign = 1 if err > 0.0 else -1
            if not meaningful_signs or meaningful_signs[-1] != sign:
                meaningful_signs.append(sign)
        meaningful_oscillations = max(0, len(meaningful_signs) - 1)
        pid_correction = p_terms[-1] + i_terms[-1] + d_terms[-1]
        feedforward_est = outputs[-1] - pid_correction

        next_kp, next_ki, next_kd = kp, ki, kd
        reasons: list[str] = []
        if is_speed_channel:
            next_kd = 0.0
            if tail_error < -tolerance:
                next_kp = kp * 1.10 if kp > 0.0 else 300.0
                next_ki = ki if ki > 0.0 else 60.0
                reasons.append("尾段速度仍高于目标：更像前馈偏高或负修正不够；优先把 SPEED_FEEDFORWARD_GAIN 降低 5%-10%，或小幅提高 Kp 增强负修正，不建议继续降低 Ki。")
            elif tail_error > tolerance and overshoot < 8.0:
                next_kp = kp * 1.10 if kp > 0.0 else 300.0
                next_ki = ki * 1.25 if ki > 0.0 else max(next_kp * 0.2, 60.0)
                reasons.append("尾段速度低于目标且无明显超调：前馈偏低或积分补偿慢；建议提高 Ki/Kp，必要时小幅提高 SPEED_FEEDFORWARD_GAIN。")
            elif tail_error > tolerance and overshoot >= 8.0:
                next_kp = kp
                next_ki = ki * 1.15 if ki > 0.0 else 60.0
                reasons.append("出现过超调但尾段又低于目标：前馈大致够，主要是稳态补偿不足；建议只小幅提高 Ki，不要加 D。")
            elif overshoot > 18.0 or meaningful_oscillations >= 3:
                next_kp *= 0.90
                next_ki *= 0.95
                reasons.append("存在明显超调/有效振荡：建议小幅降低 Kp，Ki 只轻微降低；若尾段仍偏高，优先降低前馈而不是猛压 PID。")
            elif overshoot > 10.0 and abs(tail_error) <= tolerance:
                next_kp *= 0.95
                next_ki = ki
                reasons.append("只有启动超调、尾段已接近目标：建议略降 Kp 或略降前馈，保持 Ki 负责稳态误差。")
            elif steady_error > max(0.015, step_size * 0.06):
                next_ki = ki * 1.15 if ki > 0.0 else max(kp * 0.2, 60.0)
                reasons.append("速度稳态误差偏大：建议提高 Ki，让积分补足前馈以外的 PWM。")
            else:
                reasons.append("速度响应基本可接受：建议保持 PI 参数，继续单轮验证。")
            reasons.append(f"诊断：尾段反馈≈{tail_feedback:.3f}m/s，尾段误差≈{tail_error:.3f}m/s，峰值反馈≈{peak_feedback:.3f}m/s，尾段输出≈{tail_output:.0f}PWM，估算前馈≈{feedforward_est:.0f}PWM。")
        elif overshoot > 20.0 or oscillations >= 6:
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
