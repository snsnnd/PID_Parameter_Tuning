#include "pid_debug.h"

#include <string.h>

#define PID_TX_BUF_SIZE 1024u
#define PID_RX_MAX_PAYLOAD 128u
#define PID_FRAME_OVERHEAD 10u

typedef struct {
    uint8_t data[PID_TX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} ringbuf_t;

static pid_debug_port_t g_port;
static ringbuf_t g_tx;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1u) ^ 0xA001u) : (uint16_t)(crc >> 1u);
        }
    }
    return crc;
}

static uint16_t rb_size(void) {
    return (uint16_t)((g_tx.head - g_tx.tail) % PID_TX_BUF_SIZE);
}

static uint16_t rb_free(void) {
    return (uint16_t)(PID_TX_BUF_SIZE - 1u - rb_size());
}

static int rb_push(const uint8_t *src, uint16_t len) {
    if (rb_free() < len) {
        return -1;
    }
    for (uint16_t i = 0; i < len; ++i) {
        g_tx.data[g_tx.head] = src[i];
        g_tx.head = (uint16_t)((g_tx.head + 1u) % PID_TX_BUF_SIZE);
    }
    return 0;
}

static uint16_t rb_peek_linear(uint8_t **ptr) {
    if (g_tx.head == g_tx.tail) {
        *ptr = 0;
        return 0;
    }
    *ptr = &g_tx.data[g_tx.tail];
    if (g_tx.head > g_tx.tail) {
        return (uint16_t)(g_tx.head - g_tx.tail);
    }
    return (uint16_t)(PID_TX_BUF_SIZE - g_tx.tail);
}

static void rb_drop(uint16_t len) {
    g_tx.tail = (uint16_t)((g_tx.tail + len) % PID_TX_BUF_SIZE);
}

static int send_frame(uint8_t type, uint8_t device_id, uint8_t channel_id, const uint8_t *payload, uint16_t plen) {
#if PID_DEBUG_ENABLE_TELEMETRY == 0u
    (void)type; (void)device_id; (void)channel_id; (void)payload; (void)plen;
    return 0;
#else
    uint8_t frame[PID_FRAME_OVERHEAD + PID_RX_MAX_PAYLOAD];
    uint16_t total = (uint16_t)(8u + plen + 2u);
    if (plen > PID_RX_MAX_PAYLOAD) return -2;
    frame[0] = PID_DEBUG_SOF_1;
    frame[1] = PID_DEBUG_SOF_2;
    frame[2] = PID_DEBUG_VERSION;
    frame[3] = type;
    frame[4] = device_id;
    frame[5] = channel_id;
    frame[6] = (uint8_t)(plen & 0xFFu);
    frame[7] = (uint8_t)(plen >> 8u);
    if (plen > 0u) memcpy(&frame[8], payload, plen);
    uint16_t crc = crc16_modbus(frame, (uint16_t)(8u + plen));
    frame[8u + plen] = (uint8_t)(crc & 0xFFu);
    frame[9u + plen] = (uint8_t)(crc >> 8u);
    return rb_push(frame, total);
#endif
}

void pid_debug_init(const pid_debug_port_t *port) {
    memset(&g_tx, 0, sizeof(g_tx));
    memset(&g_port, 0, sizeof(g_port));
    if (port != 0) g_port = *port;
}

int pid_debug_send_telemetry(uint8_t device_id, uint8_t channel_id, const pid_debug_telemetry_t *tel) {
    if (tel == 0) return -1;
    return send_frame(PID_FRAME_TELEMETRY, device_id, channel_id, (const uint8_t *)tel, (uint16_t)sizeof(*tel));
}

int pid_debug_send_param_report(uint8_t device_id, uint8_t channel_id, const pid_debug_param_t *param) {
    if (param == 0) return -1;
    return send_frame(PID_FRAME_PARAM_REPORT, device_id, channel_id, (const uint8_t *)param, (uint16_t)sizeof(*param));
}

int pid_debug_send_event(uint8_t device_id, uint8_t channel_id, uint16_t event_code, float value) {
    uint8_t payload[6];
    payload[0] = (uint8_t)(event_code & 0xFFu);
    payload[1] = (uint8_t)(event_code >> 8u);
    memcpy(&payload[2], &value, sizeof(float));
    return send_frame(PID_FRAME_EVENT, device_id, channel_id, payload, sizeof(payload));
}

void pid_debug_poll(void) {
#if PID_DEBUG_ENABLE_TELEMETRY == 0u
    return;
#else
    if (g_port.write == 0) return;
    uint8_t *ptr;
    uint16_t n = rb_peek_linear(&ptr);
    if (n == 0) return;
    int sent = g_port.write(ptr, n);
    if (sent > 0) rb_drop((uint16_t)sent);
#endif
}

void pid_debug_rx_byte(uint8_t byte) {
    enum { S_SOF1, S_SOF2, S_HDR, S_PAYLOAD, S_CRC1, S_CRC2 };
    static uint8_t state = S_SOF1;
    static uint8_t hdr[6];
    static uint8_t payload[PID_RX_MAX_PAYLOAD];
    static uint16_t hidx = 0, pidx = 0, plen = 0;
    static uint16_t rx_crc = 0;

    switch (state) {
        case S_SOF1: state = (byte == PID_DEBUG_SOF_1) ? S_SOF2 : S_SOF1; break;
        case S_SOF2: state = (byte == PID_DEBUG_SOF_2) ? S_HDR : S_SOF1; hidx = 0; break;
        case S_HDR:
            hdr[hidx++] = byte;
            if (hidx == sizeof(hdr)) {
                plen = (uint16_t)(hdr[4] | ((uint16_t)hdr[5] << 8u));
                if (hdr[0] != PID_DEBUG_VERSION || plen > PID_RX_MAX_PAYLOAD) {
                    state = S_SOF1;
                } else {
                    pidx = 0;
                    state = (plen == 0u) ? S_CRC1 : S_PAYLOAD;
                }
            }
            break;
        case S_PAYLOAD:
            payload[pidx++] = byte;
            if (pidx >= plen) state = S_CRC1;
            break;
        case S_CRC1: rx_crc = byte; state = S_CRC2; break;
        case S_CRC2: {
            rx_crc |= (uint16_t)(byte << 8u);
            uint8_t frame[8 + PID_RX_MAX_PAYLOAD];
            frame[0] = PID_DEBUG_SOF_1; frame[1] = PID_DEBUG_SOF_2;
            memcpy(&frame[2], hdr, sizeof(hdr));
            memcpy(&frame[8], payload, plen);
            if (crc16_modbus(frame, (uint16_t)(8 + plen)) == rx_crc) {
                if (hdr[1] == PID_FRAME_PARAM_SET && plen == sizeof(pid_debug_param_t) && g_port.on_param_set) {
                    g_port.on_param_set(hdr[2], hdr[3], (const pid_debug_param_t *)payload);
                }
            }
            state = S_SOF1;
        } break;
        default: state = S_SOF1; break;
    }
}

static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void pid_controller_reset(pid_controller_t *ctrl) {
    if (ctrl == 0) return;
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    ctrl->prev_feedback = 0.0f;
    ctrl->prev_d_term = 0.0f;
    ctrl->prev_output = 0.0f;
    ctrl->prev_ts_ms = 0u;
    ctrl->initialized = 0u;
}

void pid_controller_set_param(pid_controller_t *ctrl, const pid_debug_param_t *param) {
    if (ctrl == 0 || param == 0) return;
    ctrl->param = *param;
}

void pid_controller_init(pid_controller_t *ctrl, const pid_debug_param_t *param) {
    if (ctrl == 0) return;
    memset(ctrl, 0, sizeof(*ctrl));
    if (param != 0) ctrl->param = *param;
}

float pid_controller_update(pid_controller_t *ctrl, float target, float feedback, uint32_t now_ms) {
    if (ctrl == 0) return 0.0f;
    float dt = 0.001f;
    if (ctrl->initialized) {
        uint32_t dms = now_ms - ctrl->prev_ts_ms;
        if (dms > 0u) dt = (float)dms * 0.001f;
        if (dt > 0.5f) dt = 0.5f;
    } else {
        ctrl->initialized = 1u;
    }
    ctrl->prev_ts_ms = now_ms;

    float limited_target = target;
    if (ctrl->param.max_target_step > 0.0f) {
        limited_target = clampf(target, feedback - ctrl->param.max_target_step, feedback + ctrl->param.max_target_step);
    }
    float error = limited_target - feedback;
    if (error > -ctrl->param.deadband && error < ctrl->param.deadband) error = 0.0f;

    float p_term = ctrl->param.kp * error;
    if (ctrl->param.enable_integral) {
        ctrl->integral += error * dt;
        ctrl->integral = clampf(ctrl->integral, ctrl->param.integral_min, ctrl->param.integral_max);
    } else {
        ctrl->integral = 0.0f;
    }
    float i_term = ctrl->param.ki * ctrl->integral;

    float derivative = 0.0f;
    if (ctrl->param.enable_derivative && dt > 1e-6f) derivative = (ctrl->prev_feedback - feedback) / dt;
    float d_term = ctrl->param.kd * derivative;
    float alpha = clampf(ctrl->param.d_filter_alpha, 0.0f, 1.0f);
    d_term = alpha * ctrl->prev_d_term + (1.0f - alpha) * d_term;
    ctrl->prev_d_term = d_term;

    float ff = ctrl->param.enable_feedforward ? (ctrl->param.feedforward_gain * limited_target) : 0.0f;
    float output = p_term + i_term + d_term + ff;
    output = clampf(output, ctrl->param.output_min, ctrl->param.output_max);
    if (ctrl->param.max_output_slew_rate > 0.0f) {
        float max_step = ctrl->param.max_output_slew_rate * dt;
        output = clampf(output, ctrl->prev_output - max_step, ctrl->prev_output + max_step);
    }

    if (ctrl->param.enable_anti_windup && ctrl->param.ki != 0.0f &&
        (output <= ctrl->param.output_min || output >= ctrl->param.output_max)) {
        ctrl->integral -= error * dt;
    }
    ctrl->prev_error = error;
    ctrl->prev_feedback = feedback;
    ctrl->prev_output = output;
    return output;
}
