#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
#include "remote/remote.h"

#include <string.h>

#define RC_DMA_BUFFER_SIZE          128U
#define RC_EXPECTED_PERIOD_MS       14U
#define RC_GAP_ERROR_THRESHOLD_MS   (RC_EXPECTED_PERIOD_MS + 11U)
#define RC_TIMEOUT_MS               100U
#define RC_TASK_POLL_MS             20U

#define REMOTE_EVENT_RX             (1UL << 0)
#define REMOTE_EVENT_UART_ERROR     (1UL << 1)

static uint8_t rc_dma_buffer[RC_DMA_BUFFER_SIZE];
static uint8_t rc_frame[RC_FRAME_LENGTH];
static uint8_t rc_frame_length;
static uint32_t rc_read_count;
static volatile uint32_t rc_dma_wrap_count;
static TaskHandle_t remote_task_handle;

static TickType_t last_frame_tick;
static bool received_once;
static bool remote_link_active;

static uint16_t Remote_GetDmaWritePosition(void) {
    uint16_t write_position;

    write_position = (uint16_t)(RC_DMA_BUFFER_SIZE
        - __HAL_DMA_GET_COUNTER(huart3.hdmarx));

    if (write_position >= RC_DMA_BUFFER_SIZE) {
        write_position = 0U;
    }

    return write_position;
}

static uint32_t Remote_GetDmaWriteCount(void) {
    return (rc_dma_wrap_count * RC_DMA_BUFFER_SIZE)
        + Remote_GetDmaWritePosition();
}

static void Remote_ResetParser(void) {
    rc_frame_length = 0U;
}

static void Remote_PublishFrame(void) {
    TickType_t now = xTaskGetTickCount();

    if (received_once
        && ((now - last_frame_tick)
            > pdMS_TO_TICKS(RC_GAP_ERROR_THRESHOLD_MS))) {
        remote_frame_gap_error++;
    }

    last_frame_tick = now;
    received_once = true;
    remote_link_active = true;
    remote_rx(rc_frame);
}

static void Remote_ParseByte(uint8_t byte) {
    rc_frame[rc_frame_length++] = byte;

    if (rc_frame_length < RC_FRAME_LENGTH) {
        return;
    }

    if (remote_frame_is_valid(rc_frame)) {
        Remote_PublishFrame();
        rc_frame_length = 0U;
        return;
    }

    remote_frame_invalid_count++;
    memmove(rc_frame, &rc_frame[1], RC_FRAME_LENGTH - 1U);
    rc_frame_length = RC_FRAME_LENGTH - 1U;
}

static void Remote_ProcessDmaData(uint32_t write_count) {
    uint32_t available_count = write_count - rc_read_count;

    if (available_count > RC_DMA_BUFFER_SIZE) {
        remote_dma_overrun_count++;
        rc_read_count = write_count - RC_DMA_BUFFER_SIZE;
        Remote_ResetParser();
        available_count = RC_DMA_BUFFER_SIZE;
    }

    while (available_count > 0U) {
        Remote_ParseByte(rc_dma_buffer[rc_read_count % RC_DMA_BUFFER_SIZE]);
        rc_read_count++;
        available_count--;
    }
}

static void Remote_RecoverUart(void) {
    (void)HAL_UART_AbortReceive(&huart3);

    rc_read_count = 0U;
    rc_dma_wrap_count = 0U;
    Remote_ResetParser();
    received_once = false;
    remote_link_active = false;
    remote_clear();

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart3, rc_dma_buffer, RC_DMA_BUFFER_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void Remote_Task(void *argument) {
    uint32_t events;

    (void)argument;
    remote_task_handle = xTaskGetCurrentTaskHandle();
    remote_clear();
    Remote_ResetParser();

    if (HAL_UARTEx_ReceiveToIdle_DMA(
            &huart3, rc_dma_buffer, RC_DMA_BUFFER_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    } else {
        Remote_RecoverUart();
    }

    for (;;) {
        events = 0U;
        (void)xTaskNotifyWait(
            0U,
            UINT32_MAX,
            &events,
            pdMS_TO_TICKS(RC_TASK_POLL_MS));

        if ((events & REMOTE_EVENT_UART_ERROR) != 0U) {
            Remote_RecoverUart();
            continue;
        }

        /*
         * Size passed by HAL is intentionally ignored.  In circular mode it
         * is an event position, while NDTR is the source of truth for the
         * current DMA write position.
         */
        Remote_ProcessDmaData(Remote_GetDmaWriteCount());

        if (remote_link_active
            && ((xTaskGetTickCount() - last_frame_tick)
                > pdMS_TO_TICKS(RC_TIMEOUT_MS))) {
            remote_link_active = false;
            remote_clear();
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)Size;

    if ((huart->Instance == USART3) && (remote_task_handle != NULL)) {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC) {
            rc_dma_wrap_count++;
        }

        (void)xTaskNotifyFromISR(
            remote_task_handle,
            REMOTE_EVENT_RX,
            eSetBits,
            &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart->Instance == USART3) && (remote_task_handle != NULL)) {
        remote_uart_error_count++;
        (void)xTaskNotifyFromISR(
            remote_task_handle,
            REMOTE_EVENT_UART_ERROR,
            eSetBits,
            &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
