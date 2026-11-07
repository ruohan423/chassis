/* CubeMX 创建说明：由用户创建常驻线程。
 * 名称 power_task，入口 Power_Task，优先级 osPriorityNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；不需要额外信号量。
 * 功率模块在本任务入口初始化，不再依赖 freertos.c 中的手写初始化和创建顺序。
 */
#include "power_task.h"

#include "cmsis_os2.h"
#include "chassis.h"

#define POWER_TASK_PERIOD_MS 1U            /* 功率控制周期，单位 ms；实际周期同时用于能量环。 */
#define POWER_TASK_STARTUP_DELAY_MS 1000U  /* 功率模型初始化后的启动等待时间，单位 ms。 */

void Power_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_frequency * POWER_TASK_PERIOD_MS + 999U) / 1000U;
    chassis_power_init(osKernelGetTickCount(), tick_frequency, period_ticks);
#if STACK_TEST_ENABLED
    StackTest_Record(STACK_TEST_POWER);
#endif
    osDelay((tick_frequency * POWER_TASK_STARTUP_DELAY_MS + 999U) / 1000U);
    chassis_power_start(osKernelGetTickCount());
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        chassis_power_update(osKernelGetTickCount());
#if STACK_TEST_ENABLED
        StackTest_Record(STACK_TEST_POWER);
#endif
        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
