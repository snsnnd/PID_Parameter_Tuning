#include "pid_debug.h"

#include <string.h>

#define PID_TX_BUF_SIZE 1024u
#define PID_RX_MAX_PAYLOAD 128u
#define PID_FRAME_OVERHEAD 10u
#define PID_TELEMETRY_PAYLOAD_SIZE 54u

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

static uint16_t rb_size(void) { return (uint16_t)((g_tx.head - g_tx.tail) % PID_TX_BUF_SIZE); }
static uint16_t rb_free(void) { return (uint16_t)(PID_TX_BUF_SIZE - 1u - rb_size()); }

static int rb_push(const uint8_t *src, uint16_t len) {
    if (rb_free() < len) return -1;
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
    if (g_tx.head > g_tx.tail) return (uint16_t)(g_tx.head - g_tx.tail);
    return (uint16_t)(PID_TX_BUF_SIZE - g_tx.tail);
}

static void rb_drop(uint16_t len) { g_tx.tail = (uint16_t)((g_tx.tail + len) % PID_TX_BUF_SIZE); }

static int send_frame(uint8_t type, uint8_t device_id, uint8_t channel_id, const uint8_t *payload, uint16_t plen) {
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
}

static void put_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8u);
}

static void put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8u) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16u) & 0xFFu);
    dst[3] = (uint8_t)(value >> 24u);
}

static void put_float_le(uint8_t *dst, float value) {
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    put_u32_le(dst, raw);
}

static uint16_t encode_telemetry_payload(uint8_t *payload, const pid_debug_telemetry_t *tel) {
    uint16_t offset = 0u;
    const float values[12] = {
        tel->target, tel->feedback, tel->error, tel->output,
        tel->kp, tel->ki, tel->kd,
        tel->p_term, tel->i_term, tel->d_term,
        tel->extra1, tel->extra2
    };
    put_u32_le(&payload[offset], tel->timestamp_ms);
    offset = (uint16_t)(offset + 4u);
    for (uint8_t i = 0u; i < 12u; ++i) {
        put_float_le(&payload[offset], values[i]);
        offset = (uint16_t)(offset + 4u);
    }
    put_u16_le(&payload[offset], tel->status_flags);
    offset = (uint16_t)(offset + 2u);
    return offset;
}

void pid_debug_init(const pid_debug_port_t *port) {
    memset(&g_tx, 0, sizeof(g_tx));
    memset(&g_port, 0, sizeof(g_port));
    if (port != 0) g_port = *port;
}

int pid_debug_send_telemetry(uint8_t device_id, uint8_t channel_id, const pid_debug_telemetry_t *tel) {
    if (tel == 0) return -1;
    uint8_t payload[PID_TELEMETRY_PAYLOAD_SIZE];
    uint16_t len = encode_telemetry_payload(payload, tel);
    return send_frame(PID_FRAME_TELEMETRY, device_id, channel_id, payload, len);
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

int pid_debug_send_heartbeat(uint8_t device_id, uint16_t status_flags, uint32_t uptime_ms) {
    uint8_t payload[6];
    put_u32_le(&payload[0], uptime_ms);
    put_u16_le(&payload[4], status_flags);
    return send_frame(PID_FRAME_HEARTBEAT, device_id, 0U, payload, sizeof(payload));
}

int pid_debug_send_map_status(uint8_t device_id, uint32_t uptime_ms,
                              float distance_m, float yaw_deg,
                              uint16_t current_id, uint8_t current_type,
                              uint16_t next_id, uint8_t next_type,
                              float dist_to_next_m, uint8_t car_mode,
                              uint16_t status_flags) {
    uint8_t payload[24];
    uint16_t offset = 0U;
    put_u32_le(&payload[offset], uptime_ms); offset = (uint16_t)(offset + 4U);
    put_float_le(&payload[offset], distance_m); offset = (uint16_t)(offset + 4U);
    put_float_le(&payload[offset], yaw_deg); offset = (uint16_t)(offset + 4U);
    put_u16_le(&payload[offset], current_id); offset = (uint16_t)(offset + 2U);
    payload[offset++] = current_type;
    put_u16_le(&payload[offset], next_id); offset = (uint16_t)(offset + 2U);
    payload[offset++] = next_type;
    put_float_le(&payload[offset], dist_to_next_m); offset = (uint16_t)(offset + 4U);
    payload[offset++] = car_mode;
    put_u16_le(&payload[offset], status_flags); offset = (uint16_t)(offset + 2U);
    return send_frame(PID_FRAME_MAP_STATUS, device_id, 0U, payload, offset);
}

void pid_debug_poll(void) {
    if (g_port.write == 0) return;
    uint8_t *ptr;
    uint16_t n = rb_peek_linear(&ptr);
    if (n == 0) return;
    int sent = g_port.write(ptr, n);
    if (sent > 0) rb_drop((uint16_t)sent);
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
                if (hdr[0] != PID_DEBUG_VERSION || plen > PID_RX_MAX_PAYLOAD) state = S_SOF1;
                else { pidx = 0; state = (plen == 0u) ? S_CRC1 : S_PAYLOAD; }
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
