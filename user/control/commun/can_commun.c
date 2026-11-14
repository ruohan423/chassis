#include "can_commun.h"

#include "can.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "remote.h"
#include "task.h"
#include "user_can.h"

#include <math.h>
#include <string.h>

static gimbal_data_t gimbal_data;
static uint8_t remote_data[CAN_COMMUN_RC_DATA_LENGTH];
static bool remote_pending;
static float imu_yaw_rad;
static float imu_yaw_rate_rad_s;
static uint8_t imu_data[USER_CAN_DATA_LENGTH];
static bool imu_pending;

static void can_commun_on_receive(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    (void)handle;
    if (frame->extended || frame->id != CAN_COMMUN_GIMBAL_FEEDBACK_ID || frame->length != USER_CAN_DATA_LENGTH)
    {
        return;
    }

    const uint32_t tick = HAL_GetTick();
    const UBaseType_t saved_mask = taskENTER_CRITICAL_FROM_ISR();
    memcpy(&gimbal_data.yaw_angle_rad, &frame->data[0], sizeof(gimbal_data.yaw_angle_rad));
    memcpy(&gimbal_data.yaw_rate_rad_s, &frame->data[sizeof(gimbal_data.yaw_angle_rad)],
           sizeof(gimbal_data.yaw_rate_rad_s));
    gimbal_data.tick = tick;
    gimbal_data.online = true;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
}

bool can_commun_init(void)
{
    user_can_register_rx_callback(&hcan1, can_commun_on_receive);
    return user_can_start(&hcan1);
}

bool can_commun_read_gimbal_data(gimbal_data_t *data)
{
    const uint32_t now_tick = HAL_GetTick();

    taskENTER_CRITICAL();
    *data = gimbal_data;
    taskEXIT_CRITICAL();
    data->online = data->online && now_tick - data->tick <= CAN_COMMUN_GIMBAL_TIMEOUT_MS
        && isfinite(data->yaw_angle_rad) && isfinite(data->yaw_rate_rad_s);
    return data->online;
}

void can_commun_update_imu(float yaw_rad, float yaw_rate_rad_s)
{
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return;
    }

    imu_yaw_rad = yaw_rad;
    imu_yaw_rate_rad_s = yaw_rate_rad_s;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
}

void can_commun_send_remote(void)
{
    if (!remote_pending)
    {
        RC_Ctl_t remote;
        const int32_t previous_lock = osKernelLock();

        if (previous_lock < 0)
        {
            return;
        }

        remote = rc_data;
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }

        remote_data[0] = (uint8_t)((uint16_t)remote.rc.ch0 & 0x07FFU);
        remote_data[1] = (uint8_t)(((uint16_t)remote.rc.ch0 >> 8U)| (((uint16_t)remote.rc.ch1 & 0x07FFU) << 3U));
        remote_data[2] = (uint8_t)((((uint16_t)remote.rc.ch1 & 0x07FFU) >> 5U)| (((uint16_t)remote.rc.ch2 & 0x07FFU) << 6U));
        remote_data[3] = (uint8_t)(((uint16_t)remote.rc.ch2 & 0x07FFU) >> 2U);
        remote_data[4] = (uint8_t)((((uint16_t)remote.rc.ch2 & 0x07FFU) >> 10U)| (((uint16_t)remote.rc.ch3 & 0x07FFU) << 1U));
        remote_data[5] = (uint8_t)((((uint16_t)remote.rc.ch3 & 0x07FFU) >> 7U)| (((uint8_t)remote.rc.s2 & 0x03U) << 4U)| (((uint8_t)remote.rc.s1 & 0x03U) << 6U));
        remote_pending = true;
    }

    if (user_can_send(&hcan1, CAN_COMMUN_RC_FRAME_ID_BASE, remote_data, CAN_COMMUN_RC_DATA_LENGTH) == HAL_OK)
    {
        remote_pending = false;
    }
}

void can_commun_send_imu(void)
{
    if (!imu_pending)
    {
        const int32_t previous_lock = osKernelLock();

        if (previous_lock < 0)
        {
            return;
        }

        memcpy(imu_data, &imu_yaw_rad, sizeof(imu_yaw_rad));
        memcpy(&imu_data[sizeof(imu_yaw_rad)], &imu_yaw_rate_rad_s, sizeof(imu_yaw_rate_rad_s));
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }

        imu_pending = true;
    }

    if (user_can_send(&hcan1, CAN_COMMUN_IMU_FRAME_ID_BASE, imu_data, USER_CAN_DATA_LENGTH) == HAL_OK)
    {
        imu_pending = false;
    }
}
