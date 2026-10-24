#ifndef SENTRY_CHASSIS_REMOTE_UART_H
#define SENTRY_CHASSIS_REMOTE_UART_H

#include <stdbool.h>
#include <stdint.h>

void remote_uart_init(void);
void remote_uart_process(void);
void remote_uart_clear_state(void);
bool remote_uart_is_timeout(uint32_t timeout_ms);

#endif
