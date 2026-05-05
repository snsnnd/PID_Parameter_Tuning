#include "pid_debug.h"

void gimbal_yaw_report(float target, float feedback, float output) {
    pid_debug_telemetry_t t = {0};
    t.target = target;
    t.feedback = feedback;
    t.error = target - feedback;
    t.output = output;
    pid_debug_send_telemetry(3, 0, &t);
}
