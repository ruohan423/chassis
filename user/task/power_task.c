#include "power_task.h"

#include "cmsis_os2.h"
#include "chassis.h"

void Power_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_frequency * CHASSIS_POWER_PERIOD_MS + 999U) / 1000U;
    osDelay((tick_frequency * CHASSIS_POWER_STARTUP_DELAY_MS + 999U) / 1000U);
    chassis_power_start(osKernelGetTickCount());
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        chassis_power_update(osKernelGetTickCount());
        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
