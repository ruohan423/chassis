#include "cmsis_os2.h"

#include "user_uart.h"
#include "remote.h"
#include "usart.h"

#include <string.h>

#define REMOTE_DMA_BUFFER_SIZE 128U

extern osSemaphoreId_t RemoteSemHandle;

static uint8_t dma_buffer[REMOTE_DMA_BUFFER_SIZE];
static uint8_t frame[RC_FRAME_LENGTH];
static uint8_t frame_length;
static uint32_t read_count;
static volatile uint32_t dma_wrap_count;
static volatile bool uart_error_pending;

static uint32_t last_frame_tick;
static bool has_received_frame;

static void process_received_data(void)
{
    uint16_t write_position = (uint16_t)(REMOTE_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
    uint32_t write_count = dma_wrap_count * REMOTE_DMA_BUFFER_SIZE + (write_position < REMOTE_DMA_BUFFER_SIZE ? write_position : 0U);
    uint32_t available_count = write_count - read_count;

    if (available_count > REMOTE_DMA_BUFFER_SIZE)
    {
        remote_dma_overrun_count++;
        read_count = write_count - REMOTE_DMA_BUFFER_SIZE;
        frame_length = 0U;
        available_count = REMOTE_DMA_BUFFER_SIZE;
    }

    while (available_count > 0U)
    {
        frame[frame_length++] = dma_buffer[read_count % REMOTE_DMA_BUFFER_SIZE];
        read_count++;
        available_count--;

        if (frame_length < RC_FRAME_LENGTH)
        {
            continue;
        }

        if (remote_frame_is_valid(frame))
        {
            last_frame_tick = osKernelGetTickCount();
            has_received_frame = true;
            remote_rx(frame);
            frame_length = 0U;
        }
        else
        {
            remote_frame_invalid_count++;
            memmove(frame, &frame[1], RC_FRAME_LENGTH - 1U);
            frame_length = RC_FRAME_LENGTH - 1U;
        }
    }
}

static bool restart_uart(void)
{
    (void)HAL_UART_AbortReceive(&huart3);

    read_count = 0U;
    dma_wrap_count = 0U;
    frame_length = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_buffer, REMOTE_DMA_BUFFER_SIZE) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
        return true;
    }

    return false;
}

void remote_uart_init(void)
{
    remote_uart_clear_state();
    uart_error_pending = false;
    if (!restart_uart())
    {
        uart_error_pending = true;
        (void)osSemaphoreRelease(RemoteSemHandle);
    }
}

void remote_uart_process(void)
{
    if (uart_error_pending)
    {
        uart_error_pending = false;
        remote_uart_clear_state();
        if (!restart_uart())
        {
            uart_error_pending = true;
            (void)osSemaphoreRelease(RemoteSemHandle);
        }
        return;
    }

    process_received_data();
}

void remote_uart_clear_state(void)
{
    frame_length = 0U;
    last_frame_tick = 0U;
    has_received_frame = false;
}

bool remote_uart_is_timeout(uint32_t timeout_ms)
{
    return has_received_frame && (osKernelGetTickCount() - last_frame_tick > timeout_ms);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;

    if (huart->Instance == USART3)
    {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)
        {
            dma_wrap_count++;
        }

        (void)osSemaphoreRelease(RemoteSemHandle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_uart_error_count++;
        uart_error_pending = true;
        (void)osSemaphoreRelease(RemoteSemHandle);
    }
}
