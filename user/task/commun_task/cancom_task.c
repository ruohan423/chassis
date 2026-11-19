/* CubeMX 需创建线程 cancom_task，入口 CanCom_Task，优先级 osPriorityNormal，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128；生成句柄名称为 cancom_taskHandle。
 * 本任务使用线程标志接收有效遥控帧通知，不需要额外信号量；内核时钟要求为 1000 Hz。
 */
#include "cancom_task.h"

#include "cmsis_os2.h"
#include "can_commun.h"

#define CANCOM_SEND_PERIOD_MS   10U  /* 底盘姿态帧与云台指令帧的发送周期，单位 ms。 */
#define CANCOM_RETRY_PERIOD_MS 100U  /* CAN 总线状态检查间隔，也是启动失败或 bus-off 后的重启间隔，单位 ms。 */

static osThreadId_t cancom_task_id;

void CanCom_Task(void *argument)
{
    (void)argument;

    cancom_task_id = osThreadGetId();
    bool can_started = can_commun_init();
    uint32_t now_tick = osKernelGetTickCount();
    uint32_t next_send_tick = now_tick + CANCOM_SEND_PERIOD_MS;
    uint32_t next_retry_tick = now_tick + CANCOM_RETRY_PERIOD_MS;

    for (;;)
    {
        const uint32_t next_deadline = can_started ? next_send_tick : next_retry_tick;
        const int32_t remaining_ticks = (int32_t)(next_deadline - now_tick);
        const uint32_t wait_ticks = remaining_ticks > 0 ? (uint32_t)remaining_ticks : 0U;
        const uint32_t flags = osThreadFlagsWait(CANCOM_TASK_REMOTE_FRAME_FLAG, osFlagsWaitAny, wait_ticks);

        now_tick = osKernelGetTickCount();
        /* 按检查间隔调用 can_commun_init：总线可用时直接返回，未启动或处于 bus-off 时重启总线。 */
        if ((int32_t)(now_tick - next_retry_tick) >= 0)
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

        if ((int32_t)(now_tick - next_send_tick) >= 0)
        {
            can_commun_send_imu();
            can_commun_send_gimbal_active();
            do
            {
                next_send_tick += CANCOM_SEND_PERIOD_MS;
            } while ((int32_t)(now_tick - next_send_tick) >= 0);
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
