#include "pid_debug_adapter.h"

static void on_param_set(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param) {
    if (device_id != SMART_CAR_DEVICE_ID || param == 0) return;
    user_pid_debug_apply_param(channel_id, param);
}

void smart_car_pid_debug_init(void) {
    pid_debug_port_t port = { user_pid_debug_write, user_pid_debug_time_ms, on_param_set };
    pid_debug_init(&port);
}

void smart_car_pid_debug_rx_byte(uint8_t byte) { pid_debug_rx_byte(byte); }
void smart_car_pid_debug_poll(void) { pid_debug_poll(); }

int smart_car_pid_debug_report(uint8_t channel_id, const pid_debug_channel_sample_t *sample) {
    if (sample == 0) return -1;
    pid_debug_telemetry_t t = {0};
    t.timestamp_ms = user_pid_debug_time_ms();
    t.target = sample->target;
    t.feedback = sample->feedback;
    t.error = sample->target - sample->feedback;
    t.output = sample->output;
    t.kp = sample->kp; t.ki = sample->ki; t.kd = sample->kd;
    t.p_term = sample->p_term; t.i_term = sample->i_term; t.d_term = sample->d_term;
    t.extra1 = sample->extra1; t.extra2 = sample->extra2;
    t.status_flags = sample->status_flags;
    return pid_debug_send_telemetry(SMART_CAR_DEVICE_ID, channel_id, &t);
}

int smart_car_pid_debug_heartbeat(uint16_t status_flags) {
    return pid_debug_send_heartbeat(SMART_CAR_DEVICE_ID, status_flags, user_pid_debug_time_ms());
}
