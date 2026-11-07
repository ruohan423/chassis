#include "user_can.h"

#include <string.h>

static user_can_rx_callback_t can1_rx_callback;
static user_can_rx_callback_t can2_rx_callback;

static bool configure_filter(CAN_HandleTypeDef *handle)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.SlaveStartFilterBank = 14U;
    filter.FilterBank = handle->Instance == CAN1 ? 0U : 14U;
    filter.FilterFIFOAssignment = handle->Instance == CAN1 ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    filter.FilterActivation = ENABLE;
    return HAL_CAN_ConfigFilter(handle, &filter) == HAL_OK;
}

static void process_received_frames(CAN_HandleTypeDef *handle, uint32_t fifo)
{
    while (HAL_CAN_GetRxFifoFillLevel(handle, fifo) > 0U)
    {
        CAN_RxHeaderTypeDef header = {0};
        user_can_frame_t frame = {0};

        if (HAL_CAN_GetRxMessage(handle, fifo, &header, frame.data) != HAL_OK){return;}

        frame.extended = header.IDE == CAN_ID_EXT;
        frame.id = frame.extended ? header.ExtId : header.StdId;
        frame.length = (uint8_t)header.DLC;

        if (handle->Instance == CAN1 && can1_rx_callback != NULL) can1_rx_callback(handle, &frame);
        else if (handle->Instance == CAN2 && can2_rx_callback != NULL) can2_rx_callback(handle, &frame);
    }
}

bool user_can_start(CAN_HandleTypeDef *handle)
{
    uint32_t notification;

    if (HAL_CAN_GetState(handle) == HAL_CAN_STATE_LISTENING){return true;}

    if (!configure_filter(handle) || HAL_CAN_Start(handle) != HAL_OK){return false;}

    notification = handle->Instance == CAN1  ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;
    if (HAL_CAN_ActivateNotification(handle, notification) != HAL_OK)
    {
        (void)HAL_CAN_Stop(handle);
        return false;
    }
    return true;
}

bool user_can_stop(CAN_HandleTypeDef *handle)
{
    if (HAL_CAN_GetState(handle) != HAL_CAN_STATE_LISTENING){return true;}
    const uint32_t notification = handle->Instance == CAN1
        ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;
    if (HAL_CAN_DeactivateNotification(handle, notification) != HAL_OK
        || HAL_CAN_Stop(handle) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool user_can_is_started(const CAN_HandleTypeDef *handle)
{
    return HAL_CAN_GetState(handle) == HAL_CAN_STATE_LISTENING;
}

void user_can_register_rx_callback(CAN_HandleTypeDef *handle, user_can_rx_callback_t callback)
{
    if (handle->Instance == CAN1)
    {
        can1_rx_callback = callback;
    }
    else if (handle->Instance == CAN2)
    {
        can2_rx_callback = callback;
    }
}

HAL_StatusTypeDef user_can_send(CAN_HandleTypeDef *handle, uint32_t standard_id,const uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef header = {
        .StdId = standard_id,
        .ExtId = 0U,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = length,
        .TransmitGlobalTime = DISABLE,
    };
    uint8_t payload[USER_CAN_DATA_LENGTH] = {0};
    uint32_t mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0U){return HAL_BUSY;}

    memcpy(payload, data, length);
    return HAL_CAN_AddTxMessage(handle, &header, payload, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    if (handle->Instance == CAN1)
    {
        process_received_frames(handle, CAN_RX_FIFO0);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *handle)
{
    if (handle->Instance == CAN2)
    {
        process_received_frames(handle, CAN_RX_FIFO1);
    }
}
