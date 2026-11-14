/* CubeMX 需创建线程 cancom_task，入口 CanCom_Task，优先级 osPriorityNormal，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128；生成句柄名称为 cancom_taskHandle。
 * 本任务使用线程标志接收有效遥控帧通知，不需要额外信号量；内核时钟要求为 1000 Hz。
 */
#include "cancom_task.h"

#include "cmsis_os2.h"
#include "can_commun.h"

#define CANCOM_IMU_PERIOD_MS    10U  /* 底盘 IMU CAN 帧发送周期，单位 ms。 */
#define CANCOM_RETRY_PERIOD_MS 100U  /* CAN 启动失败后的重试间隔，单位 ms。 */

static osThreadId_t cancom_task_id;

void CanCom_Task(void *argument)
{
    (void)argument;

    cancom_task_id = osThreadGetId();
    bool can_started = can_commun_init();
    uint32_t now_tick = osKernelGetTickCount();
    uint32_t next_imu_tick = now_tick + CANCOM_IMU_PERIOD_MS;
    uint32_t next_retry_tick = now_tick + CANCOM_RETRY_PERIOD_MS;

    for (;;)
    {
        const uint32_t next_deadline = can_started ? next_imu_tick : next_retry_tick;
        const int32_t remaining_ticks = (int32_t)(next_deadline - now_tick);
        const uint32_t wait_ticks = remaining_ticks > 0 ? (uint32_t)remaining_ticks : 0U;
        const uint32_t flags = osThreadFlagsWait(CANCOM_TASK_REMOTE_FRAME_FLAG, osFlagsWaitAny, wait_ticks);

        now_tick = osKernelGetTickCount();
        if (!can_started && (int32_t)(now_tick - next_retry_tick) >= 0)
        {
            can_started = can_commun_init();
            next_retry_tick += CANCOM_RETRY_PERIOD_MS;
            while ((int32_t)(now_tick - next_retry_tick) >= 0)
            {
                next_retry_tick += CANCOM_RETRY_PERIOD_MS;
            }
        }
        if (!can_started)
        {
            continue;
        }

        if ((int32_t)(now_tick - next_imu_tick) >= 0)
        {
            can_commun_send_imu();
            do
            {
                next_imu_tick += CANCOM_IMU_PERIOD_MS;
            } while ((int32_t)(now_tick - next_imu_tick) >= 0);
        }
        if ((flags & CANCOM_TASK_REMOTE_FRAME_FLAG) != 0U)
        {
            can_commun_send_remote();
        }
    }
}

void CanCom_TaskNotifyRemoteFrame(void)
{
    if (cancom_task_id != NULL)
    {
        (void)osThreadFlagsSet(cancom_task_id, CANCOM_TASK_REMOTE_FRAME_FLAG);
    }
}
