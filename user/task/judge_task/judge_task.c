/* CubeMX 创建说明：由用户创建常驻线程及二值信号量。
 * 线程名称 judge_task，入口 Judge_Task，优先级 osPriorityNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；本任务要求内核时钟为 1000 Hz，1 tick 对应 1ms。
 * 二值信号量名称 JudgeSem，最大值 1、初值 0，生成句柄必须为 JudgeSemHandle。
 * 复用 USART6：RX DMA Circular、TX DMA Normal；USART/DMA 中断优先级为 5。
 */
#include "judge_task.h"

#include "cmsis_os2.h"
#include "judge.h"
#include "user_uart.h"
#include "usart.h"

#define JUDGE_DMA_BUFFER_SIZE 1024U
#define JUDGE_SEMAPHORE_WAIT_MS 10U
#define JUDGE_FRAME_TIMEOUT_MS 50U
#define JUDGE_TX_INTERVAL_MS ((1000U + JUDGE_TX_MAX_HZ - 1U) / JUDGE_TX_MAX_HZ)  /* 按 0x0301 发送频率上限向上取整的最小帧间隔，单位 ms。 */

static uint8_t dma_buffer[JUDGE_DMA_BUFFER_SIZE];
static uint8_t received_data[JUDGE_DMA_BUFFER_SIZE];

uart_rx_t judge_uart_rx = {
    .handle = &huart6,
    .buffer = dma_buffer,
    .size = JUDGE_DMA_BUFFER_SIZE,
};

static void process_transmit(uint32_t now_tick);

void Judge_Task(void *argument)
{
    (void)argument;

    uint32_t last_byte_tick = 0U;
    uint32_t last_frame_tick = 0U;
    bool has_valid_frame = false;

    judge_clear();
    bool uart_started = uart_rx_start(&judge_uart_rx);
#if STACK_TEST_ENABLED
    StackTest_Record(STACK_TEST_JUDGE);
#endif

    for (;;)
    {
        (void)osSemaphoreAcquire(JudgeSemHandle, JUDGE_SEMAPHORE_WAIT_MS);
        const uint32_t now_tick = osKernelGetTickCount();

        if (!uart_started || judge_uart_rx.error_pending)
        {
            judge_clear();
            has_valid_frame = false;
            uart_started = uart_rx_start(&judge_uart_rx);
        }
        else
        {
            bool overrun;
            const uint16_t received_length = uart_rx_read(&judge_uart_rx, received_data, &overrun);
            if (overrun)
            {
                judge_clear();
                has_valid_frame = false;
            }
            if (received_length == 0U && now_tick - last_byte_tick >= JUDGE_FRAME_TIMEOUT_MS)
            {
                judge_reset_parser();
            }
            if (received_length > 0U)
            {
                if (judge_receive(received_data, received_length, now_tick))
                {
                    last_frame_tick = now_tick;
                    has_valid_frame = true;
                }
                last_byte_tick = now_tick;
            }
            if (has_valid_frame && now_tick - last_frame_tick >= JUDGE_LINK_TIMEOUT_MS)
            {
                judge_clear();
                has_valid_frame = false;
            }
        }

        process_transmit(osKernelGetTickCount());
#if STACK_TEST_ENABLED
        StackTest_Record(STACK_TEST_JUDGE);
#endif
    }
}

static void process_transmit(uint32_t now_tick)
{
    static uint8_t frame[JUDGE_FRAME_MAX_SIZE];
    static uint32_t start_tick;

    if (huart6.gState != HAL_UART_STATE_READY)
    {
        /* 发送完成中断丢失或 DMA 卡住时中止本帧，否则后续帧永远无法启动。 */
        if (now_tick - start_tick >= JUDGE_TX_TIMEOUT_MS)
        {
            (void)HAL_UART_AbortTransmit(&huart6);
        }
        return;
    }
    if (now_tick - start_tick < JUDGE_TX_INTERVAL_MS)
    {
        return;
    }

    const uint16_t length = judge_pack_tx(frame, now_tick);
    if (length > 0U && HAL_UART_Transmit_DMA(&huart6, frame, length) == HAL_OK)
    {
        start_tick = now_tick;
    }
}
