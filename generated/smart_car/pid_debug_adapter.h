#ifndef PID_DEBUG_ADAPTER_H
#define PID_DEBUG_ADAPTER_H

#include "pid_debug.h"
#include "pid_debug_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target;
    float feedback;
    float output;
    float kp;
    float ki;
    float kd;
    float p_term;
    float i_term;
    float d_term;
    float extra1;
    float extra2;
    uint16_t status_flags;
} pid_debug_channel_sample_t;

int user_pid_debug_write(const uint8_t *data, uint16_t len);
uint32_t user_pid_debug_time_ms(void);
void user_pid_debug_apply_param(uint8_t channel_id, const pid_debug_param_t *param);

void smart_car_pid_debug_init(void);
void smart_car_pid_debug_rx_byte(uint8_t byte);
void smart_car_pid_debug_poll(void);
int smart_car_pid_debug_report(uint8_t channel_id, const pid_debug_channel_sample_t *sample);
int smart_car_pid_debug_heartbeat(uint16_t status_flags);

#ifdef __cplusplus
}
#endif

#endif
