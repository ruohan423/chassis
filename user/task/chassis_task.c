#include "chassis_task.h"
#include "cmsis_os2.h"

#include "chassis.h"
#include "remote.h"

void Chassis_Task(void *argument)
{
    (void)argument;

    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t period_ticks = (tick_frequency * CHASSIS_CONTROL_PERIOD_MS + 999U) / 1000U;
    const uint32_t retry_ticks = (tick_frequency * CHASSIS_CAN_RETRY_MS + 999U) / 1000U;
    bool ready = chassis_init(osKernelGetTickCount(), tick_frequency);
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!ready)
        {
            osDelay(retry_ticks);
            ready = chassis_init(osKernelGetTickCount(), tick_frequency);
            next_tick = osKernelGetTickCount();
            continue;
        }

        const float input_x = (float)(rc_data.rc.ch2 - RC_STICK_OFFSET) * CHASSIS_STICK_X_DIRECTION / CHASSIS_STICK_DIVISOR;
        const float input_y = (float)(rc_data.rc.ch3 - RC_STICK_OFFSET) * CHASSIS_STICK_Y_DIRECTION / CHASSIS_STICK_DIVISOR;
        /* 云台相对角来源尚未接入；跟随模式通过有效标志保持安全停止。 */
        ready = chassis_update(input_x, input_y, 0.0f, false, osKernelGetTickCount());

        next_tick += period_ticks;
        osDelayUntil(next_tick);
    }
}
