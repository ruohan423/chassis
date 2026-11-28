#ifndef SENTRY_CHASSIS_UART_TEST_H
#define SENTRY_CHASSIS_UART_TEST_H

#include <stdint.h>

/* 仅由 UartCom_Task 调用，代替 uart_send_update()；每次调用最多起发一帧模拟数据包。 */
void uart_test_send_update(uint32_t now_tick);

#endif
