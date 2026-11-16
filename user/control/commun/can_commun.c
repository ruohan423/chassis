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
static uint8_t remote_data[CAN_COMMUN_REMOTE_LENGTH];
static bool remote_pending;
static float imu_yaw_rad;
static float imu_yaw_rate_rad_s;
static uint8_t imu_data[CAN_COMMUN_IMU_LENGTH];
static bool imu_pending;
static gimbal_active_t gimbal_active;
static uint32_t gimbal_active_tick;

static void can_commun_on_receive(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    (void)handle;
    if (frame->extended || frame->id != CAN_COMMUN_GIMBAL_FEEDBACK_ID || frame->length != CAN_COMMUN_GIMBAL_FEEDBACK_LENGTH)
    {
        return;
    }

    int16_t yaw_angle;
    int16_t yaw_rate;
    memcpy(&yaw_angle, &frame->data[0], sizeof(yaw_angle));
    memcpy(&yaw_rate, &frame->data[2], sizeof(yaw_rate));
    const uint32_t tick = HAL_GetTick();
    const UBaseType_t saved_mask = taskENTER_CRITICAL_FROM_ISR();
    gimbal_data.yaw_angle_rad = (float)yaw_angle * CAN_COMMUN_ANGLE_LSB_RAD;
    gimbal_data.yaw_rate_rad_s = (float)yaw_rate * CAN_COMMUN_RATE_LSB_RAD_S;
    gimbal_data.pitch_status = (frame->data[4] & 0x01U) != 0U;
    gimbal_data.energy_mechanis_done = (frame->data[4] & 0x02U) != 0U;
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
    /* 当前时刻与快照在同一临界区内读取，避免接收中断在两者之间刷新时间戳而被误判为超时。 */
    taskENTER_CRITICAL();
    const uint32_t now_tick = HAL_GetTick();
    *data = gimbal_data;
    taskEXIT_CRITICAL();
    data->online = data->online && now_tick - data->tick <= CAN_COMMUN_GIMBAL_TIMEOUT_MS;
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

void can_commun_update_gimbal_active(const gimbal_active_t *active)
{
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return;
    }

    gimbal_active = *active;
    gimbal_active_tick = osKernelGetTickCount();
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
}

void can_commun_send_remote(void)
{
    if (!remote_pending)
    {
        static RC_Ctl_t remote_snapshot;
        const int32_t previous_lock = osKernelLock();

        if (previous_lock < 0)
        {
            return;
        }

        remote_snapshot = rc_data;
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }

        remote_data[0] = (uint8_t)((uint16_t)remote_snapshot.rc.ch0 & 0x07FFU);
        remote_data[1] = (uint8_t)(((uint16_t)remote_snapshot.rc.ch0 >> 8U)
            | (((uint16_t)remote_snapshot.rc.ch1 & 0x07FFU) << 3U));
        remote_data[2] = (uint8_t)((((uint16_t)remote_snapshot.rc.ch1 & 0x07FFU) >> 5U)
            | (((uint16_t)remote_snapshot.rc.ch2 & 0x07FFU) << 6U));
        remote_data[3] = (uint8_t)(((uint16_t)remote_snapshot.rc.ch2 & 0x07FFU) >> 2U);
        remote_data[4] = (uint8_t)((((uint16_t)remote_snapshot.rc.ch2 & 0x07FFU) >> 10U)
            | (((uint16_t)remote_snapshot.rc.ch3 & 0x07FFU) << 1U));
        remote_data[5] = (uint8_t)((((uint16_t)remote_snapshot.rc.ch3 & 0x07FFU) >> 7U)
            | (((uint8_t)remote_snapshot.rc.s2 & 0x03U) << 4U)
            | (((uint8_t)remote_snapshot.rc.s1 & 0x03U) << 6U));
        remote_pending = true;
    }

    if (user_can_send(&hcan1, CAN_COMMUN_REMOTE_ID, remote_data, CAN_COMMUN_REMOTE_LENGTH) == HAL_OK)
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

        const float yaw_rad = imu_yaw_rad;
        const float yaw_rate_rad_s = imu_yaw_rate_rad_s;
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }

        /* yaw 来自欧拉角，本身在 -π～π 内；角速度按陀螺仪量程可能超出 int16 表示范围，需饱和。 */
        const int16_t yaw = (int16_t)lroundf(yaw_rad / CAN_COMMUN_ANGLE_LSB_RAD);
        const int16_t yaw_rate = (int16_t)lroundf(fmaxf(-32767.0f, fminf(yaw_rate_rad_s / CAN_COMMUN_RATE_LSB_RAD_S, 32767.0f)));
        memcpy(&imu_data[0], &yaw, sizeof(yaw));
        memcpy(&imu_data[2], &yaw_rate, sizeof(yaw_rate));
        imu_pending = true;
    }

    if (user_can_send(&hcan1, CAN_COMMUN_IMU_ID, imu_data, CAN_COMMUN_IMU_LENGTH) == HAL_OK)
    {
        imu_pending = false;
    }
}

void can_commun_send_gimbal_active(void)
{
    static uint8_t active_frame[CAN_COMMUN_GIMBAL_ACTIVE_LENGTH];
    static gimbal_active_t active_snapshot;
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return;
    }

    active_snapshot = gimbal_active;
    const bool timed_out = osKernelGetTickCount() - gimbal_active_tick > CAN_COMMUN_GIMBAL_ACTIVE_TIMEOUT_MS;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }

    /* 上电未收到指令或上位机指令超时：按全零默认发送，即升起云台、跟随底盘、关闭能量机关。 */
    if (timed_out)
    {
        active_snapshot = (gimbal_active_t){0};
    }

    const int16_t target_angle = (int16_t)lroundf(remainderf(active_snapshot.gimabal_tar_angle, 6.28318530718f)
        / CAN_COMMUN_ANGLE_LSB_RAD);
    const uint16_t patrol_angle = (uint16_t)lroundf(fmaxf(0.0f,
        fminf(active_snapshot.gimabal_pat_angle / CAN_COMMUN_ANGLE_LSB_RAD, 65535.0f)));
    active_frame[0] = (uint8_t)((active_snapshot.pitch_cmd ? 0x01U : 0U)
        | (active_snapshot.activate_energy ? 0x02U : 0U)
        | ((active_snapshot.gimabal_model & 0x03U) << 2U));
    memcpy(&active_frame[1], &target_angle, sizeof(target_angle));
    memcpy(&active_frame[3], &patrol_angle, sizeof(patrol_angle));
    /* 周期状态帧不重试：邮箱忙时丢弃本帧，下个周期发送最新指令。 */
    (void)user_can_send(&hcan1, CAN_COMMUN_GIMBAL_ACTIVE_ID, active_frame, CAN_COMMUN_GIMBAL_ACTIVE_LENGTH);
}
