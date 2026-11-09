#include "can_common.h"

#include "can.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "ins_task.h"
#include "remote.h"
#include "task.h"
#include "user_can.h"

#include <string.h>

static can_common_yaw_data_t yaw_data;
static uint32_t remote_ids[3];
static uint8_t remote_data[3][USER_CAN_DATA_LENGTH];
static uint8_t remote_index;
static bool remote_pending;
static uint32_t imu_ids[3];
static uint8_t imu_data[3][USER_CAN_DATA_LENGTH];
static uint8_t imu_index;
static bool imu_pending;

static void can_common_on_receive(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    (void)handle;
    if (frame->extended || frame->id != CAN_COMMON_YAW_FEEDBACK_ID || frame->length != USER_CAN_DATA_LENGTH)
    {
        return;
    }

    const uint32_t tick = HAL_GetTick();
    const UBaseType_t saved_mask = taskENTER_CRITICAL_FROM_ISR();
    yaw_data.angle = (uint16_t)((frame->data[0] << 8) | frame->data[1]);
    yaw_data.speed = (int16_t)((frame->data[2] << 8) | frame->data[3]);
    yaw_data.current = (int16_t)((frame->data[4] << 8) | frame->data[5]);
    yaw_data.temperature = frame->data[6];
    yaw_data.tick = tick;
    yaw_data.online = true;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
}

bool can_common_init(void)
{
    user_can_register_rx_callback(&hcan1, can_common_on_receive);
    return user_can_start(&hcan1);
}

bool can_common_read_yaw_data(can_common_yaw_data_t *data)
{
    const uint32_t now_tick = HAL_GetTick();

    taskENTER_CRITICAL();
    *data = yaw_data;
    taskEXIT_CRITICAL();
    data->online = data->online && now_tick - data->tick <= CAN_COMMON_YAW_TIMEOUT_MS;
    return data->online;
}

void can_common_send_remote(void)
{
    RC_Ctl_t remote;
    const int32_t previous_lock = osKernelLock();

    if (previous_lock < 0)
    {
        return;
    }

    if (remote_pending)
    {
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        return;
    }

    remote = rc_data;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }

    remote_ids[0] = CAN_COMMON_RC_FRAME_ID_BASE;
    remote_data[0][0] = (uint8_t)remote.rc.ch0;
    remote_data[0][1] = (uint8_t)((uint16_t)remote.rc.ch0 >> 8U);
    remote_data[0][2] = (uint8_t)remote.rc.ch1;
    remote_data[0][3] = (uint8_t)((uint16_t)remote.rc.ch1 >> 8U);
    remote_data[0][4] = (uint8_t)remote.rc.ch2;
    remote_data[0][5] = (uint8_t)((uint16_t)remote.rc.ch2 >> 8U);
    remote_data[0][6] = (uint8_t)remote.rc.ch3;
    remote_data[0][7] = (uint8_t)((uint16_t)remote.rc.ch3 >> 8U);

    remote_ids[1] = CAN_COMMON_RC_FRAME_ID_BASE + 1U;
    remote_data[1][0] = (uint8_t)remote.rc.s1;
    remote_data[1][1] = (uint8_t)remote.rc.s2;
    remote_data[1][2] = (uint8_t)remote.mouse.x;
    remote_data[1][3] = (uint8_t)((uint16_t)remote.mouse.x >> 8U);
    remote_data[1][4] = (uint8_t)remote.mouse.y;
    remote_data[1][5] = (uint8_t)((uint16_t)remote.mouse.y >> 8U);
    remote_data[1][6] = (uint8_t)remote.mouse.z;
    remote_data[1][7] = (uint8_t)((uint16_t)remote.mouse.z >> 8U);

    remote_ids[2] = CAN_COMMON_RC_FRAME_ID_BASE + 2U;
    remote_data[2][0] = remote.mouse.press_l;
    remote_data[2][1] = remote.mouse.press_r;
    remote_data[2][2] = remote.mouse.last_press_l;
    remote_data[2][3] = remote.mouse.last_press_r;
    remote_data[2][4] = (uint8_t)remote.key.value;
    remote_data[2][5] = (uint8_t)(remote.key.value >> 8U);
    remote_data[2][6] = (uint8_t)remote.lastkey.value;
    remote_data[2][7] = (uint8_t)(remote.lastkey.value >> 8U);
    remote_index = 0U;
    remote_pending = true;
    can_common_process();
}

void can_common_send_imu(void)
{
    ins_output_t imu;
    const int32_t previous_lock = osKernelLock();

    if (previous_lock < 0)
    {
        return;
    }

    if (imu_pending)
    {
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        return;
    }

    imu.roll_deg = ins_output.roll_deg;
    imu.pitch_deg = ins_output.pitch_deg;
    imu.yaw_deg = ins_output.yaw_deg;
    imu.accel_earth_x = ins_output.accel_earth_x;
    imu.accel_earth_y = ins_output.accel_earth_y;
    imu.accel_earth_z = ins_output.accel_earth_z;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }

    imu_ids[0] = CAN_COMMON_IMU_FRAME_ID_BASE;
    memcpy(imu_data[0], &imu.roll_deg, sizeof(imu.roll_deg));
    memcpy(&imu_data[0][4], &imu.pitch_deg, sizeof(imu.pitch_deg));
    imu_ids[1] = CAN_COMMON_IMU_FRAME_ID_BASE + 1U;
    memcpy(imu_data[1], &imu.yaw_deg, sizeof(imu.yaw_deg));
    memcpy(&imu_data[1][4], &imu.accel_earth_x, sizeof(imu.accel_earth_x));
    imu_ids[2] = CAN_COMMON_IMU_FRAME_ID_BASE + 2U;
    memcpy(imu_data[2], &imu.accel_earth_y, sizeof(imu.accel_earth_y));
    memcpy(&imu_data[2][4], &imu.accel_earth_z, sizeof(imu.accel_earth_z));
    imu_index = 0U;
    imu_pending = true;
    can_common_process();
}

void can_common_process(void)
{
    while (imu_pending && imu_index < 3U)
    {
        if (user_can_send(&hcan1, imu_ids[imu_index], imu_data[imu_index], USER_CAN_DATA_LENGTH) != HAL_OK)
        {
            return;
        }
        imu_index++;
    }

    imu_pending = false;
    while (remote_pending && remote_index < 3U)
    {
        if (user_can_send(&hcan1, remote_ids[remote_index], remote_data[remote_index], USER_CAN_DATA_LENGTH) != HAL_OK)
        {
            return;
        }
        remote_index++;
    }

    remote_pending = false;
}
