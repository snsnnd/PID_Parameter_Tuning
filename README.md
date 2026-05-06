# PID_Parameter_Tuning / PIDScope Offline

离线多设备 PID 调参工具（MVP），包括：

- Python 桌面端（PySide6 + pyqtgraph）
- 二进制协议解析 + CRC16
- 多设备/多通道缓冲（`device_id + channel_id`）
- 实时波形（target/feedback/error/output）
- 基础阶跃分析（超调、稳态误差、振荡计数、IAE）

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
  - `examples/motor_speed_example.c`：电机速度环
  - `examples/gimbal_yaw_angle_loop_example.c`：陀螺仪旋转角度环
  - `examples/line_tracking_position_loop_example.c`：循迹位置环（含航向辅助环）
  - `examples/cascaded_angle_position_loop_example.c`：角度/位置外环 + 角速度内环一体化示例
  - 支持 `PID_DEBUG_ENABLE_TELEMETRY` 宏开关（置 `0` 可禁用日志帧发送）
  - 内置 `pid_controller_*` API，可直接作为控制算法调用
