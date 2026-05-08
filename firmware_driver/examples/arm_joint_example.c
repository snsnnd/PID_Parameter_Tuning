#include "pid_debug.h"

void arm_joint_report(uint8_t joint, float target, float feedback, float torque) {
    pid_debug_telemetry_t t = {0};
    t.target = target;
    t.feedback = feedback;
    t.error = target - feedback;
    t.extra1 = torque;
    pid_debug_send_telemetry(4 + joint, 0, &t);
}
