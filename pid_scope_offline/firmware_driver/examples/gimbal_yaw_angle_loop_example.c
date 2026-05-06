#include "pid_debug.h"

/*
 * 陀螺仪旋转控制（角度环）示例：
 * - device_id = 5, channel_id = 0
 * - target/feedback 以“角度（deg）”上报
 * - output 可上报“角速度指令”或“电机电流指令”
 */
void gimbal_yaw_angle_report(float target_angle_deg,
                             float measured_angle_deg,
                             float yaw_rate_cmd,
                             float gyro_z_dps) {
    pid_debug_telemetry_t t = {0};
    t.target = target_angle_deg;
    t.feedback = measured_angle_deg;
    t.error = target_angle_deg - measured_angle_deg;
    t.output = yaw_rate_cmd;
    t.extra1 = gyro_z_dps;  // 额外通道：Z 轴陀螺仪角速度（deg/s）

    pid_debug_send_telemetry(5, 0, &t);
}
