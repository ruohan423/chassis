/* CubeMX 创建说明：保留或由用户创建看门狗常驻线程。
 * 名称 watchdog_task，入口 Watchdog_Task，优先级 osPriorityLow，栈 512 字节，生成句柄 watchdog_taskHandle。
 * 若界面以 32 位字填写栈大小，填 128；不需要额外信号量，内核时钟为 1000 Hz。
 */
#include "watch_task.h"

#include "board_led.h"
#include "cmsis_os2.h"
#include "watchdog.h"
#include "watchdog_control.h"

#define WATCHDOG_TASK_PERIOD_MS 2U

void Watchdog_Task(void *argument)
{
    uint32_t next_tick;
    (void)argument;

#if defined(HAL_IWDG_MODULE_ENABLED)
    (void)watchdog_iwdg_init();
#endif
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        const bool remote_expired = watchdog_control_is_expired(WATCH_REMOTE);
        const bool imu_expired = watchdog_control_is_expired(WATCH_IMU);
        const bool host_expired = watchdog_control_is_expired(WATCH_HOST);

        /* 遥控链路超时的清零与安全行为由 remote_task 处理，此处只驱动指示灯 */
        board_led_set(BOARD_LED_BLUE, !remote_expired);
        /* 上位机链路仅指示：收到有效上位机帧时绿灯亮；导航和云台指令的失效保护在各自所有者处。 */
        board_led_set(BOARD_LED_GREEN, !host_expired);
        /* 姿态任务活动指示让出绿灯，改用红灯：亮表示 INS_Task 在按周期采样。 */
        board_led_set(BOARD_LED_RED, !imu_expired);

#if defined(HAL_IWDG_MODULE_ENABLED)
        (void)watchdog_iwdg_refresh();
#endif

        next_tick += WATCHDOG_TASK_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
