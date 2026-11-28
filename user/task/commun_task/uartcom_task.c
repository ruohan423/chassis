/* CubeMX 创建说明：由 CubeMX 创建 uartcom_task 常驻线程和 UartComSem 二值信号量。
 * 线程入口 UartCom_Task，优先级 osPriorityLow，栈 512 字节；CMSIS-RTOS2 stack_size 单位为字节。
 * UartComSem 最大值 1、初值 0，句柄 UartComSemHandle；USART1 RX Circular DMA、TX Normal DMA，中断优先级 5。
 * USB 通信源码保留；当前调度配置使用本任务处理 USART1 收发。
 */
#include "uartcom_task.h"

#include "cmsis_os2.h"
#include "uart_commun.h"
#include "watchdog_control.h"

#define UART_TASK_PERIOD_MS 2U
#define UART_RX_FRAME_TIMEOUT_MS 50U

#define UART_SEND_TEST 0  /* 1：发送 uart_test.c 的模拟数据包，联调上位机协议用；0：正式发送。接收处理两者相同。 */

#if UART_SEND_TEST
#include "uart_test.h"
#endif

void UartCom_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_freq = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_freq * UART_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t frame_timeout_ticks = (tick_freq * UART_RX_FRAME_TIMEOUT_MS + 999U) / 1000U;
    uart_commun_reset();
    bool uart_started = uart_rx_start(&host_uart_rx);
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        (void)osSemaphoreAcquire(UartComSemHandle, period_ticks);
        const uint32_t now_tick = osKernelGetTickCount();
        if (!uart_started || host_uart_rx.error_pending)
        {
            uart_commun_reset();
            uart_started = uart_rx_start(&host_uart_rx);
        }
        else
        {
            if (uart_receive_process(now_tick, frame_timeout_ticks))
            {
                watchdog_control_feed(WATCH_HOST);
            }
        }

#if UART_SEND_TEST
        uart_test_send_update(now_tick);
#else
        uart_send_update(now_tick);
#endif
        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
