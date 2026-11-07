/* CubeMX 创建说明：由用户创建常驻线程，不在 freertos.c 用户区手写创建代码。
 * 名称 chassis_task，入口 Chassis_Task，优先级 osPriorityAboveNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；入口应绑定本文件实现，避免重复强定义。
 * 无独立信号量；本任务内初始化底盘，启用功率控制时，功率任务完成首轮计算前保持零输出。
 */
#include "chassis_task.h"
#include "cmsis_os2.h"

#include "chassis.h"
#include "remote.h"

#define CHASSIS_TASK_PERIOD_MS 2U    /* 底盘控制周期，单位 ms；实际周期同时用于轮速和跟随 PID。 */
#define CHASSIS_START_RETRY_MS 100U  /* 总线初始化或发送失败后的重试间隔，单位 ms。 */

void Chassis_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_frequency * CHASSIS_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t retry_ticks = (tick_frequency * CHASSIS_START_RETRY_MS + 999U) / 1000U;
    bool ready = chassis_init(osKernelGetTickCount(), tick_frequency, period_ticks);
#if STACK_TEST_ENABLED
    StackTest_Record(STACK_TEST_CHASSIS);
#endif
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!ready)
        {
            osDelay(retry_ticks);
            ready = chassis_init(osKernelGetTickCount(), tick_frequency, period_ticks);
#if STACK_TEST_ENABLED
            StackTest_Record(STACK_TEST_CHASSIS);
#endif
            next_tick = osKernelGetTickCount();
            continue;
        }

        const float input_x = (float)(rc_data.rc.ch2 - RC_STICK_OFFSET) * CHASSIS_STICK_X_DIRECTION / CHASSIS_STICK_DIVISOR;
        const float input_y = (float)(rc_data.rc.ch3 - RC_STICK_OFFSET) * CHASSIS_STICK_Y_DIRECTION / CHASSIS_STICK_DIVISOR;
        /* 云台相对角来源尚未接入；跟随模式通过有效标志保持安全停止。 */
        ready = chassis_update(input_x, input_y, 0.0f, false, osKernelGetTickCount());
#if STACK_TEST_ENABLED
        StackTest_Record(STACK_TEST_CHASSIS);
#endif

        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
