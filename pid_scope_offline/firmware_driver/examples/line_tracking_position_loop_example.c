#include "pid_debug.h"

/*
 * 循迹模块位置环示例：
 * - device_id = 6
 * - channel_id = 0: 横向偏移（line offset）位置环
 * - channel_id = 1: 航向角（heading）辅助环（可选）
 */
void line_tracking_offset_report(float target_offset,
                                 float measured_offset,
                                 float steering_cmd,
                                 float line_confidence) {
    pid_debug_telemetry_t t = {0};
    t.target = target_offset;
    t.feedback = measured_offset;
    t.error = target_offset - measured_offset;
    t.output = steering_cmd;
    t.extra1 = line_confidence; // 额外通道：识别置信度(0~1)

    pid_debug_send_telemetry(6, 0, &t);
}

void line_tracking_heading_report(float target_heading_deg,
                                  float measured_heading_deg,
                                  float yaw_cmd) {
    pid_debug_telemetry_t t = {0};
    t.target = target_heading_deg;
    t.feedback = measured_heading_deg;
    t.error = target_heading_deg - measured_heading_deg;
    t.output = yaw_cmd;

    pid_debug_send_telemetry(6, 1, &t);
}
