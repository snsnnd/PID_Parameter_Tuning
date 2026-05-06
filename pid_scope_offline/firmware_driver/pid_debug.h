#ifndef PID_DEBUG_H
#define PID_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_DEBUG_VERSION 0x01u
#define PID_DEBUG_SOF_1   0xA5u
#define PID_DEBUG_SOF_2   0x5Au
#ifndef PID_DEBUG_ENABLE_TELEMETRY
#define PID_DEBUG_ENABLE_TELEMETRY 1u
#endif

typedef enum {
    PID_FRAME_TELEMETRY    = 0x01,
    PID_FRAME_PARAM_SET    = 0x02,
    PID_FRAME_PARAM_REPORT = 0x03,
    PID_FRAME_DEVICE_INFO  = 0x04,
    PID_FRAME_EVENT        = 0x05,
    PID_FRAME_HEARTBEAT    = 0x06,
    PID_FRAME_ERROR        = 0x07
} pid_frame_type_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float deadband;
    float d_filter_alpha;
    float feedforward_gain;
    float max_target_step;
    float max_output_slew_rate;
    uint8_t enable_integral;
    uint8_t enable_derivative;
    uint8_t enable_anti_windup;
    uint8_t enable_feedforward;
} pid_debug_param_t;

typedef struct {
    uint32_t timestamp_ms;
    float target;
    float feedback;
    float error;
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
} pid_debug_telemetry_t;

typedef int (*pid_debug_write_fn)(const uint8_t *data, uint16_t len);
typedef uint32_t (*pid_debug_time_fn)(void);
typedef void (*pid_debug_param_set_cb)(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param);

typedef struct {
    pid_debug_write_fn write;
    pid_debug_time_fn get_time_ms;
    pid_debug_param_set_cb on_param_set;
} pid_debug_port_t;

typedef struct {
    pid_debug_param_t param;
    float integral;
    float prev_error;
    float prev_feedback;
    float prev_d_term;
    float prev_output;
    uint32_t prev_ts_ms;
    uint8_t initialized;
} pid_controller_t;

void pid_debug_init(const pid_debug_port_t *port);
int pid_debug_send_telemetry(uint8_t device_id, uint8_t channel_id, const pid_debug_telemetry_t *tel);
int pid_debug_send_param_report(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param);
int pid_debug_send_event(uint8_t device_id, uint8_t channel_id, uint16_t event_code, float value);
void pid_debug_rx_byte(uint8_t byte);
void pid_debug_poll(void);

void pid_controller_init(pid_controller_t *ctrl, const pid_debug_param_t *param);
void pid_controller_set_param(pid_controller_t *ctrl, const pid_debug_param_t *param);
void pid_controller_reset(pid_controller_t *ctrl);
float pid_controller_update(pid_controller_t *ctrl, float target, float feedback, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
