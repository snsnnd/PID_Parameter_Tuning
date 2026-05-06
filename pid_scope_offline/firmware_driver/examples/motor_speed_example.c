#include "pid_debug.h"

extern int uart_dma_send(const uint8_t *data, uint16_t len);
extern uint32_t system_get_ms(void);

static pid_debug_param_t g_speed_param;

static int debug_write(const uint8_t *data, uint16_t len) { return uart_dma_send(data, len); }
static uint32_t debug_time(void) { return system_get_ms(); }

static void on_param_set(uint8_t dev, uint8_t ch, const pid_debug_param_t *param) {
    if (dev == 1 && ch == 0 && param) g_speed_param = *param;
}

void motor_debug_init(void) {
    pid_debug_port_t port = { debug_write, debug_time, on_param_set };
    pid_debug_init(&port);
}
