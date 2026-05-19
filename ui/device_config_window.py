import json
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox, QTextEdit,
    QLabel, QMessageBox
)

from core.device_schema import CONFIG_DIR, list_schema_files, load_schema, parse_schema, save_schema
from core.driver_generator import generate_driver


class DeviceConfigWindow(QWidget):
    config_saved = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("设备/地面站参数配置")
        self.resize(960, 700)
        self.current_path: Path | None = None
        self._build_ui()
        self.reload_files()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        self.file_combo = QComboBox()
        self.reload_btn = QPushButton("重新加载")
        self.new_btn = QPushButton("新建设备")
        self.save_btn = QPushButton("保存配置")
        self.generate_btn = QPushButton("生成下位机驱动")
        top.addWidget(QLabel("设备配置"))
        top.addWidget(self.file_combo, 1)
        top.addWidget(self.reload_btn)
        top.addWidget(self.new_btn)
        top.addWidget(self.save_btn)
        top.addWidget(self.generate_btn)
        self.editor = QTextEdit()
        self.editor.setPlaceholderText("编辑设备 JSON。ground_params 可增删地面站参数，channels 可增删通道。")
        layout.addLayout(top)
        layout.addWidget(self.editor)

        self.file_combo.currentIndexChanged.connect(self.load_selected)
        self.reload_btn.clicked.connect(self.reload_files)
        self.new_btn.clicked.connect(self.new_device)
        self.save_btn.clicked.connect(self.save_current)
        self.generate_btn.clicked.connect(self.generate_current)

    def reload_files(self):
        self.file_combo.blockSignals(True)
        self.file_combo.clear()
        for path in list_schema_files():
            self.file_combo.addItem(path.name, str(path))
        self.file_combo.blockSignals(False)
        if self.file_combo.count() > 0:
            self.file_combo.setCurrentIndex(0)
            self.load_selected()

    def load_selected(self):
        path_text = self.file_combo.currentData()
        if not path_text:
            return
        self.current_path = Path(path_text)
        self.editor.setPlainText(self.current_path.read_text(encoding="utf-8"))

    def new_device(self):
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)
        path = CONFIG_DIR / "new_device.json"
        idx = 1
        while path.exists():
            idx += 1
            path = CONFIG_DIR / f"new_device_{idx}.json"
        data = {
            "device": {"id": 10, "name": path.stem, "display_name": "新设备"},
            "transport": {"default_baud": 115200, "telemetry_period_ms": 100},
            "channels": [{"id": 0, "name": "main", "display_name": "主通道", "kind": "pid"}],
            "ground_params": [{"name": "目标值", "key": "target", "default": "0", "command": "TARGET {value}", "type": "float", "description": "示例参数"}],
            "commands": [{"name": "停止", "command": "STOP"}],
            "status_flags": {"0": "运行"}
        }
        save_schema(data, path)
        self.reload_files()
        index = self.file_combo.findData(str(path))
        if index >= 0:
            self.file_combo.setCurrentIndex(index)

    def _read_json(self) -> dict:
        try:
            return json.loads(self.editor.toPlainText())
        except json.JSONDecodeError as exc:
            raise ValueError(f"JSON 格式错误：{exc}") from exc

    def save_current(self):
        if not self.current_path:
            QMessageBox.warning(self, "保存配置", "没有选择配置文件。")
            return
        try:
            data = self._read_json()
            parse_schema(data, self.current_path.stem)
            save_schema(data, self.current_path)
        except Exception as exc:
            QMessageBox.warning(self, "保存配置", str(exc))
            return
        self.config_saved.emit()
        QMessageBox.information(self, "保存配置", "已保存。重新加载主窗口配置后生效。")

    def generate_current(self):
        if not self.current_path:
            QMessageBox.warning(self, "生成驱动", "没有选择配置文件。")
            return
        try:
            self.save_current()
            schema = load_schema(self.current_path)
            out_dir = generate_driver(schema)
        except Exception as exc:
            QMessageBox.warning(self, "生成驱动", str(exc))
            return
        QMessageBox.information(self, "生成驱动", f"已生成到：{out_dir}")
