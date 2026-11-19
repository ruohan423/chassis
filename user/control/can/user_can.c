#include "user_can.h"

#include <string.h>

#include "cmsis_os2.h"

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

    /* 两路 CAN 的过滤器寄存器都在 CAN1 上，由各自的任务在启动和恢复时改写；
     * 这里只有几次寄存器读改写，用调度锁避免两个任务交错。未获得保护时不配置，按启动失败处理。 */
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return false;
    }
    const bool configured = HAL_CAN_ConfigFilter(handle, &filter) == HAL_OK;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
    return configured;
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
    const uint32_t notification = handle->Instance == CAN1 ? CAN_IT_RX_FIFO0_MSG_PENDING : CAN_IT_RX_FIFO1_MSG_PENDING;

    if (user_can_is_online(handle)){return true;}

    /* 首次启动、启动失败后的重试和 bus-off 后的软件恢复共用以下流程。
     * HAL_CAN_Init 让控制器重新进入初始化模式，并把停在 ERROR 的句柄恢复为 READY；HAL_CAN_Start 退出初始化模式后，
     * 控制器在总线上监测到 128 组连续 11 个隐性位才离开 bus-off。
     * 先关接收中断：启动失败后句柄停在 ERROR，控制器却会在总线空闲后自行退出初始化模式并开始接收，
     * 此时接收回调取不走 FIFO 中的帧，中断会反复进入；中断在本函数成功返回前重新打开。
     * 邮箱里滞留的旧帧在重启时中止，不在总线恢复后补发。 */
    __HAL_CAN_DISABLE_IT(handle, notification);
    if (HAL_CAN_Init(handle) != HAL_OK){return false;}
    (void)HAL_CAN_AbortTxRequest(handle, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    if (!configure_filter(handle) || HAL_CAN_Start(handle) != HAL_OK){return false;}

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

bool user_can_is_online(const CAN_HandleTypeDef *handle)
{
    return HAL_CAN_GetState(handle) == HAL_CAN_STATE_LISTENING && (handle->Instance->ESR & CAN_ESR_BOFF) == 0U;
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

    if (handle == NULL || data == NULL || standard_id > 0x7FFU || length > USER_CAN_DATA_LENGTH){return HAL_ERROR;}
    /* bus-off 期间不入邮箱，避免旧帧滞留到总线恢复后才发出。 */
    if (!user_can_is_online(handle)){return HAL_ERROR;}
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
