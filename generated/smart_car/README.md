# 智能车 PID Debug Adapter

Generated adapter for device `smart_car`.

## Fixed User API

Implement these functions in your firmware:

```c
int user_pid_debug_write(const uint8_t *data, uint16_t len);
uint32_t user_pid_debug_time_ms(void);
void user_pid_debug_apply_param(uint8_t channel_id, const pid_debug_param_t *param);
```

## Channels

- `0` 循迹PID (line_pid)
- `1` 左轮速度PID (left_speed)
- `2` 右轮速度PID (right_speed)

Copy `pid_debug.c/.h`, `pid_debug_config.h`, and `pid_debug_adapter.c/.h` into your firmware project.
