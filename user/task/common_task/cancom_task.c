/* CubeMX 需创建线程 cancom_task，入口 CanCom_Task，优先级 osPriorityNormal，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128；生成句柄名称为 cancom_taskHandle。
 * 本任务使用线程标志接收有效遥控帧通知，不需要额外信号量；内核时钟要求为 1000 Hz。
 */
#include "cancom_task.h"

#include "cmsis_os2.h"
#include "can_common.h"

#define CANCOM_IMU_PERIOD_MS 10U
#define CANCOM_RETRY_PERIOD_MS 100U

static osThreadId_t cancom_task_id;

void CanCom_Task(void *argument)
{
    (void)argument;

    cancom_task_id = osThreadGetId();
    bool can_started = can_common_init();
    uint32_t next_imu_tick = osKernelGetTickCount() + CANCOM_IMU_PERIOD_MS;
    uint32_t next_retry_tick = osKernelGetTickCount() + CANCOM_RETRY_PERIOD_MS;
    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();
        uint32_t next_wake_tick = next_imu_tick;
        if (!can_started && (int32_t)(next_retry_tick - next_wake_tick) < 0)
        {
            next_wake_tick = next_retry_tick;
        }
        const uint32_t wait_ticks = (int32_t)(next_wake_tick - now_tick) > 0
            ? next_wake_tick - now_tick : 0U;
        const uint32_t flags = osThreadFlagsWait(CANCOM_TASK_REMOTE_FRAME_FLAG, osFlagsWaitAny, wait_ticks);

        const uint32_t tick = osKernelGetTickCount();
        if (!can_started && (int32_t)(tick - next_retry_tick) >= 0)
        {
            can_started = can_common_init();
            next_retry_tick += CANCOM_RETRY_PERIOD_MS;
        }
        if (!can_started)
        {
            if ((int32_t)(tick - next_imu_tick) >= 0)
            {
                next_imu_tick += CANCOM_IMU_PERIOD_MS;
            }
            continue;
        }

        can_common_process();
        if ((int32_t)(tick - next_imu_tick) >= 0)
        {
            can_common_send_imu();
            next_imu_tick += CANCOM_IMU_PERIOD_MS;
            while ((int32_t)(tick - next_imu_tick) >= 0)
            {
                next_imu_tick += CANCOM_IMU_PERIOD_MS;
            }
        }
        if ((flags & CANCOM_TASK_REMOTE_FRAME_FLAG) != 0U)
        {
            can_common_send_remote();
        }
        can_common_process();
    }
}

void CanCom_TaskNotifyRemoteFrame(void)
{
    if (cancom_task_id != NULL)
    {
        (void)osThreadFlagsSet(cancom_task_id, CANCOM_TASK_REMOTE_FRAME_FLAG);
    }
}
