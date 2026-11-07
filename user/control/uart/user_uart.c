#include "user_uart.h"

static uint32_t dma_write_count(const uart_rx_t *rx);

bool uart_rx_start(uart_rx_t *rx)
{
    (void)HAL_UART_AbortReceive(rx->handle);
    rx->read_count = 0U;
    rx->wrap_count = 0U;
    rx->error_pending = false;

    /* 不在运行时覆盖 CubeMX 配置，配置未完成时交给任务周期重试。 */
    if (rx->handle->hdmarx == NULL || rx->handle->hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        return false;
    }

    return HAL_UARTEx_ReceiveToIdle_DMA(rx->handle, rx->buffer, rx->size) == HAL_OK;
}

uint16_t uart_rx_read(uart_rx_t *rx, uint8_t *buffer, bool *overrun)
{
    const uint32_t write_count = dma_write_count(rx);
    const uint32_t available = write_count - rx->read_count;
    *overrun = false;

    if (available > rx->size)
    {
        rx->read_count = write_count;
        *overrun = true;
        return 0U;
    }

    for (uint32_t index = 0U; index < available; index++)
    {
        buffer[index] = ((volatile uint8_t *)rx->buffer)[(rx->read_count + index) % rx->size];
    }

    /* 复制期间 DMA 仍在写；已被追上的数据不能继续送入解析器。 */
    const uint32_t latest_write_count = dma_write_count(rx);
    if (latest_write_count - rx->read_count > rx->size || rx->error_pending)
    {
        rx->read_count = latest_write_count;
        *overrun = true;
        return 0U;
    }

    rx->read_count += available;
    return (uint16_t)available;
}

bool uart_tx_is_configured(const uart_tx_t *tx)
{
    return tx->handle->hdmatx != NULL && tx->handle->hdmatx->Init.Mode == DMA_NORMAL;
}

HAL_StatusTypeDef uart_tx_start(uart_tx_t *tx, const uint8_t *buffer, uint16_t length)
{
    if (!uart_tx_is_configured(tx))
    {
        return HAL_ERROR;
    }
    tx->completed = false;
    tx->error_pending = false;
    const HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(tx->handle, buffer, length);
    if (result == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(tx->handle->hdmatx, DMA_IT_HT);
    }
    return result;
}

bool uart_tx_abort(uart_tx_t *tx)
{
    if (HAL_UART_AbortTransmit(tx->handle) != HAL_OK)
    {
        return false;
    }
    tx->completed = false;
    tx->error_pending = false;
    return true;
}

static uint32_t dma_write_count(const uart_rx_t *rx)
{
    uint32_t wraps;
    uint32_t pending;
    uint16_t position;

    /* TC 已置位但回调尚未执行时也要计入一次环绕；重读排除跨越 TC 的采样。 */
    do
    {
        wraps = rx->wrap_count;
        pending = __HAL_DMA_GET_FLAG(rx->handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(rx->handle->hdmarx));
        position = (uint16_t)(rx->size - __HAL_DMA_GET_COUNTER(rx->handle->hdmarx));
    } while (wraps != rx->wrap_count
        || pending != __HAL_DMA_GET_FLAG(rx->handle->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(rx->handle->hdmarx)));

    if (pending != 0U || position == rx->size)
    {
        wraps++;
    }

    return wraps * rx->size + (position < rx->size ? position : 0U);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;

    if (huart == remote_uart_rx.handle)
    {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)
        {
            remote_uart_rx.wrap_count++;
        }
        (void)osSemaphoreRelease(RemoteSemHandle);
    }
    else if (huart == judge_uart_rx.handle)
    {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)
        {
            judge_uart_rx.wrap_count++;
        }
        (void)osSemaphoreRelease(JudgeSemHandle);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == remote_uart_rx.handle)
    {
        remote_uart_rx.error_pending = true;
        (void)osSemaphoreRelease(RemoteSemHandle);
    }
    else if (huart == judge_uart_rx.handle)
    {
        judge_uart_rx.error_pending = true;
        judge_uart_tx.error_pending = true;
        (void)osSemaphoreRelease(JudgeSemHandle);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == judge_uart_tx.handle)
    {
        judge_uart_tx.completed = true;
        (void)osSemaphoreRelease(JudgeSemHandle);
    }
}
