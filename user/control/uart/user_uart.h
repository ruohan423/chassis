#ifndef SENTRY_CHASSIS_USER_UART_H
#define SENTRY_CHASSIS_USER_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

typedef struct
{
    UART_HandleTypeDef *handle;
    uint8_t *buffer;
    uint16_t size;                 /* 缓冲长度取 2 的幂，使累计字节数回绕后仍与 DMA 位置一致。 */
    uint32_t read_count;
    volatile uint32_t wrap_count;
    volatile bool error_pending;
} uart_rx_t;

extern uart_rx_t remote_uart_rx;
extern uart_rx_t judge_uart_rx;
extern osSemaphoreId_t RemoteSemHandle;
extern osSemaphoreId_t JudgeSemHandle;

/* DMA 必须已在生成配置中设为 Circular，保留 HT/TC 中断用于持续流接收。 */
bool uart_rx_start(uart_rx_t *rx);
/* 调用方 buffer 至少为 rx->size 字节；溢出时丢弃待处理字节并要求上层复位解析。 */
uint16_t uart_rx_read(uart_rx_t *rx, uint8_t *buffer, bool *overrun);

#endif
