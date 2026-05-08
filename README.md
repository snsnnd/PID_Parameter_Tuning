# PID_Parameter_Tuning / PIDScope Offline

离线多设备 PID 调参工具（MVP），包括：

- Python 桌面端（PySide6 + pyqtgraph）
- 二进制协议解析 + CRC16
- 多设备/多通道缓冲（`device_id + channel_id`）
- 实时波形（target/feedback/error/output）
- 基础阶跃分析（超调、稳态误差、振荡计数、IAE）
- 增益调度 PID 建议（按电池电压分段，默认读取 telemetry.extra1）

## 快速开始

```bash
cd pid_scope_offline
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python app.py
```

## 目录

见 `pid_scope_offline/` 下分层：

- `core/`: 协议、解析、缓存、分析
- `transport/`: 串口接收线程
- `ui/`: 主窗口与交互
- `config/`: 默认配置

- `firmware_driver/`: 已实现 `pid_debug.h/.c`（非阻塞环形缓冲 + CRC + PARAM_SET 解析）与示例

## 增益调度配置

在 `pid_scope_offline/config/default_config.json` 中通过 `gain_schedule.voltage_bands` 配置分段区间与对应 `kp/ki/kd`。

> 约定：`telemetry.extra1` 为电池电压（V）。


## Advanced Tuning Lab（独立高级页）

主界面顶部 `Advanced Tuning` 按钮可打开独立高级调参页，包含：

- 分段切换防抖
- 插值模式切换（linear/step/smoothstep）
- 一键写参（PARAM_SET）
- LADRC 实时建议与 IAE 热力图
- 无硬件模拟（启动模拟）
