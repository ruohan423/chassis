/* CubeMX 创建说明：由用户创建常驻线程及二值信号量。
 * 线程名称 judge_task，入口 Judge_Task，优先级 osPriorityNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512。
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

typedef struct
{
    uint32_t tick;
    uint16_t length;
} judge_tx_record_t;

static uint8_t dma_buffer[JUDGE_DMA_BUFFER_SIZE];
static uint8_t received_data[JUDGE_DMA_BUFFER_SIZE];
uart_rx_t judge_uart_rx = {
    .handle = &huart6,
    .buffer = dma_buffer,
    .size = JUDGE_DMA_BUFFER_SIZE,
};
uart_tx_t judge_uart_tx = {.handle = &huart6};

static void process_transmit(uint32_t now_tick, uint32_t tick_frequency);

void Judge_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t wait_ticks = (tick_frequency * JUDGE_SEMAPHORE_WAIT_MS + 999U) / 1000U;
    const uint32_t frame_timeout_ticks = (tick_frequency * JUDGE_FRAME_TIMEOUT_MS + 999U) / 1000U;
    const uint32_t link_timeout_ticks = (tick_frequency * JUDGE_LINK_TIMEOUT_MS + 999U) / 1000U;
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
        (void)osSemaphoreAcquire(JudgeSemHandle, wait_ticks);
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
            if (received_length == 0U && now_tick - last_byte_tick >= frame_timeout_ticks)
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
            if (has_valid_frame && now_tick - last_frame_tick >= link_timeout_ticks)
            {
                judge_clear();
                has_valid_frame = false;
            }
        }

        process_transmit(osKernelGetTickCount(), tick_frequency);
#if STACK_TEST_ENABLED
        StackTest_Record(STACK_TEST_JUDGE);
#endif
    }
}

static void process_transmit(uint32_t now_tick, uint32_t tick_frequency)
{
    static judge_tx_request_t request;
    static uint8_t buffer[JUDGE_FRAME_MAX_SIZE];
    static uint16_t length;
    static uint8_t sequence;
    static bool pending;
    static bool active;
    static uint32_t start_tick;
    static judge_tx_record_t history[JUDGE_TX_MAX_HZ];
    static uint8_t history_head;
    static uint8_t history_count;
    static uint16_t window_bytes;

    const uint32_t timeout_ticks = (tick_frequency * JUDGE_TX_TIMEOUT_MS + 999U) / 1000U;
    if (active)
    {
        const bool aborting = judge_tx_status.result == JUDGE_TX_DMA_ERROR || judge_tx_status.result == JUDGE_TX_TIMEOUT;
        if (!aborting && judge_uart_tx.completed && !judge_uart_tx.error_pending)
        {
            judge_uart_tx.completed = false;
            active = false;
            pending = false;
            judge_tx_status.result = JUDGE_TX_DONE;
            return;
        }
        if (!aborting)
        {
            if (judge_uart_tx.error_pending || judge_uart_rx.error_pending)
            {
                judge_tx_status.result = JUDGE_TX_DMA_ERROR;
            }
            else if (now_tick - start_tick >= timeout_ticks)
            {
                judge_tx_status.result = JUDGE_TX_TIMEOUT;
            }
            else
            {
                return;
            }
        }
        /* 中止失败时保留原故障和 DMA 缓冲区，后续唤醒继续中止，不能复用其内容。 */
        if (uart_tx_abort(&judge_uart_tx))
        {
            active = false;
            pending = false;
        }
        return;
    }

    if (!uart_tx_is_configured(&judge_uart_tx))
    {
        judge_tx_status.result = JUDGE_TX_CONFIG_ERROR;
        return;
    }
    if (judge_tx_status.result == JUDGE_TX_CONFIG_ERROR || judge_tx_status.result == JUDGE_TX_NOT_READY)
    {
        judge_tx_status.result = JUDGE_TX_IDLE;
    }
    if (!pending)
    {
        if (!judge_take_tx(&request))
        {
            return;
        }
        pending = true;
        const int32_t previous_lock = osKernelLock();
        judge_tx_status.result = JUDGE_TX_QUEUED;
        judge_tx_status.data_cmd_id = request.data_cmd_id;
        judge_tx_status.receiver_id = request.receiver_id;
        judge_tx_status.sequence = sequence;
        if (previous_lock == 0)
        {
            (void)osKernelUnlock();
        }
    }

    const judge_tx_result_t result = judge_prepare_tx(&request, sequence, now_tick, buffer, &length);
    if (result != JUDGE_TX_READY)
    {
        judge_tx_status.result = result;
        pending = false;
        return;
    }

    /* 滚动一秒窗口同时限制帧数和字节数，不在窗口边界补发积压流量。 */
    while (history_count > 0U && now_tick - history[history_head].tick >= tick_frequency)
    {
        window_bytes -= history[history_head].length;
        history_head = (history_head + 1U) % JUDGE_TX_MAX_HZ;
        history_count--;
    }
    const uint32_t interval_ticks = (tick_frequency + JUDGE_TX_MAX_HZ - 1U) / JUDGE_TX_MAX_HZ;
    const uint32_t byte_limit = judge_tx_status.byte_limit_per_second < huart6.Init.BaudRate / 10U
        ? judge_tx_status.byte_limit_per_second : huart6.Init.BaudRate / 10U;
    if (history_count == JUDGE_TX_MAX_HZ || (uint32_t)window_bytes + length > byte_limit
        || (history_count > 0U && now_tick - start_tick < interval_ticks))
    {
        return;
    }

    const HAL_StatusTypeDef transmit_result = uart_tx_start(&judge_uart_tx, buffer, length);
    if (transmit_result == HAL_OK)
    {
        active = true;
        sequence++;
        start_tick = osKernelGetTickCount();
        history[(history_head + history_count) % JUDGE_TX_MAX_HZ] = (judge_tx_record_t){start_tick, length};
        history_count++;
        window_bytes += length;
        judge_tx_status.result = JUDGE_TX_ACTIVE;
    }
    else if (transmit_result == HAL_BUSY)
    {
        judge_tx_status.result = JUDGE_TX_UART_BUSY;
    }
    else
    {
        judge_tx_status.result = JUDGE_TX_DMA_ERROR;
        /* 启动尚未成功，可以在请求有效期内重试；成功启动后的异常不自动重发。 */
        if (!uart_tx_abort(&judge_uart_tx))
        {
            active = true;
            judge_uart_tx.error_pending = true;
        }
    }
}
