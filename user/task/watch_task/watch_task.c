/* CubeMX 创建说明：保留或由用户创建看门狗常驻线程。
 * 名称 watchdog_task，入口 Watchdog_Task，优先级 osPriorityLow，栈 512 字节。
 * 若界面以 32 位字填写栈大小，填 128；不需要额外信号量，内核时钟为 1000 Hz。
 */
#include "watch_task.h"

#include "board_led.h"
#include "cmsis_os2.h"
#include "watchdog.h"
#define WATCHDOG_TASK_PERIOD_MS 2U
#define REMOTE_WATCHDOG_TIMEOUT_MS 100U
#define IMU_WATCHDOG_TIMEOUT_MS 100U
#define USB_WATCHDOG_TIMEOUT_MS 100U
#define JUDGE_WATCHDOG_TIMEOUT_MS 100U
#define OP_WATCHDOG_TIMEOUT_MS 60U
#define MOTOR_WATCHDOG_TIMEOUT_MS 40U

typedef enum
{
    WATCH_REMOTE = 0,
    WATCH_IMU,
    WATCH_USB,
    WATCH_JUDGE,
    WATCH_OP,
    WATCH_CHASSIS_1,
    WATCH_CHASSIS_2,
    WATCH_CHASSIS_3,
    WATCH_CHASSIS_4,
    WATCH_CHASSIS_TRACK,
    WATCH_GIMBAL_YAW,
    WATCH_COUNT
} watch_source_t;

static watchdog_swdg_t software_watchdogs[WATCH_COUNT];

void Watchdog_Task(void *argument)
{
    uint32_t next_tick;
    (void)argument;

    watchdog_swdg_init(&software_watchdogs[WATCH_REMOTE], REMOTE_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_IMU], IMU_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_USB], USB_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_JUDGE], JUDGE_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_OP], OP_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_CHASSIS_1], MOTOR_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_CHASSIS_2], MOTOR_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_CHASSIS_3], MOTOR_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_CHASSIS_4], MOTOR_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_CHASSIS_TRACK], MOTOR_WATCHDOG_TIMEOUT_MS);
    watchdog_swdg_init(&software_watchdogs[WATCH_GIMBAL_YAW], MOTOR_WATCHDOG_TIMEOUT_MS);
#if defined(HAL_IWDG_MODULE_ENABLED)
    (void)watchdog_iwdg_init();
#endif
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        const bool remote_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_REMOTE]);
        const bool imu_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_IMU]);
        // const bool usb_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_USB]);
        // const bool judge_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_JUDGE]);
        // const bool op_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_OP]);
        // const bool chassis_1_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_CHASSIS_1]);
        // const bool chassis_2_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_CHASSIS_2]);
        // const bool chassis_3_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_CHASSIS_3]);
        // const bool chassis_4_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_CHASSIS_4]);
        // const bool chassis_track_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_CHASSIS_TRACK]);
        // const bool yaw_expired = watchdog_swdg_is_expired(&software_watchdogs[WATCH_GIMBAL_YAW]);

        /* 遥控链路超时的清零与安全行为由 remote_task 处理，此处只驱动指示灯 */
        board_led_set(BOARD_LED_BLUE, !remote_expired);

        //board_led_set(BOARD_LED_RED, !chassis_1_expired && !chassis_2_expired
        //    && !chassis_3_expired && !chassis_4_expired);
        board_led_set(BOARD_LED_GREEN, !imu_expired);


#if defined(HAL_IWDG_MODULE_ENABLED)
        (void)watchdog_iwdg_refresh();
#endif

        next_tick += WATCHDOG_TASK_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}

void swdg_feed_remote(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_REMOTE]);
}

void swdg_feed_imu(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_IMU]);
}

void swdg_feed_usb(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_USB]);
}

void swdg_feed_judge(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_JUDGE]);
}

void swdg_feed_op(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_OP]);
}

void swdg_feed_chassis_1(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_CHASSIS_1]);
}

void swdg_feed_chassis_2(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_CHASSIS_2]);
}

void swdg_feed_chassis_3(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_CHASSIS_3]);
}

void swdg_feed_chassis_4(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_CHASSIS_4]);
}

void swdg_feed_chassis_track(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_CHASSIS_TRACK]);
}

void swdg_feed_yaw(void)
{
    watchdog_swdg_feed(&software_watchdogs[WATCH_GIMBAL_YAW]);
}
