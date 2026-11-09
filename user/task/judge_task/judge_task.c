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
#define JUDGE_TX_WINDOW_MS 1000U  /* 滚动限频窗口，单位 ms。 */
#define JUDGE_TX_INTERVAL_MS ((JUDGE_TX_WINDOW_MS + JUDGE_TX_MAX_HZ - 1U) / JUDGE_TX_MAX_HZ)  /* 按发送频率上限向上取整的最小间隔，单位 ms。 */

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
    static judge_tx_frame_t frame;
    static uint16_t length;
    static uint8_t sequence;
    static bool active;
    static uint32_t start_tick;
    static judge_tx_record_t history[JUDGE_TX_MAX_HZ];
    static uint8_t history_head;
    static uint8_t history_count;
    static uint16_t window_bytes;

    if (active)
    {
        if (judge_tx_status.result != JUDGE_TX_DMA_ERROR && judge_tx_status.result != JUDGE_TX_TIMEOUT)
        {
            if (judge_uart_tx.completed && !judge_uart_tx.error_pending)
            {
                judge_uart_tx.completed = false;
                active = false;
                length = 0U;
                judge_tx_status.result = JUDGE_TX_DONE;
                return;
            }
            if (judge_uart_tx.error_pending || judge_uart_rx.error_pending)
            {
                judge_tx_status.result = JUDGE_TX_DMA_ERROR;
            }
            else if (now_tick - start_tick >= JUDGE_TX_TIMEOUT_MS)
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
            length = 0U;
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
    if (length == 0U)
    {
        if (!judge_take_tx(&frame))
        {
            return;
        }
        const int32_t previous_lock = osKernelLock();
        judge_tx_status.result = JUDGE_TX_QUEUED;
        judge_tx_status.data_cmd_id = frame.data_cmd_id;
        judge_tx_status.receiver_id = frame.receiver_id;
        judge_tx_status.sequence = sequence;
        if (previous_lock == 0)
        {
            (void)osKernelUnlock();
        }
    }

    const judge_tx_result_t result = judge_prepare_tx(&frame, sequence, now_tick, &length);
    if (result != JUDGE_TX_READY)
    {
        judge_tx_status.result = result;
        length = 0U;
        return;
    }

    /* 滚动一秒窗口同时限制帧数和字节数，不在窗口边界补发积压流量。 */
    while (history_count > 0U && now_tick - history[history_head].tick >= JUDGE_TX_WINDOW_MS)
    {
        window_bytes -= history[history_head].length;
        history_head = (history_head + 1U) % JUDGE_TX_MAX_HZ;
        history_count--;
    }
    const uint32_t byte_limit = judge_tx_status.byte_limit_per_second < huart6.Init.BaudRate / 10U
        ? judge_tx_status.byte_limit_per_second : huart6.Init.BaudRate / 10U;
    if (history_count == JUDGE_TX_MAX_HZ || (uint32_t)window_bytes + length > byte_limit
        || (history_count > 0U && now_tick - start_tick < JUDGE_TX_INTERVAL_MS))
    {
        return;
    }

    switch (uart_tx_start(&judge_uart_tx, frame.buffer, length))
    {
        case HAL_OK:
            active = true;
            sequence++;
            start_tick = osKernelGetTickCount();
            history[(history_head + history_count) % JUDGE_TX_MAX_HZ] = (judge_tx_record_t){start_tick, length};
            history_count++;
            window_bytes += length;
            judge_tx_status.result = JUDGE_TX_ACTIVE;
            break;
        case HAL_BUSY:
            judge_tx_status.result = JUDGE_TX_UART_BUSY;
            break;
        default:
            judge_tx_status.result = JUDGE_TX_DMA_ERROR;
            /* 启动尚未成功，可以在请求有效期内重试；成功启动后的异常不自动重发。 */
            if (!uart_tx_abort(&judge_uart_tx))
            {
                active = true;
                judge_uart_tx.error_pending = true;
            }
            break;
    }
}
