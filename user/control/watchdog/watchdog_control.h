#ifndef SENTRY_CHASSIS_WATCHDOG_CONTROL_H
#define SENTRY_CHASSIS_WATCHDOG_CONTROL_H

#include <stdbool.h>

typedef enum
{
    WATCH_REMOTE = 0, /* 有效 DBUS 帧，Remote_Task 喂狗并处置超时 */
    WATCH_IMU,        /* 姿态任务采样活动，INS_Task 喂狗，仅指示 */
    WATCH_JUDGE,      /* 任一有效裁判帧，Judge_Task 喂狗并处置超时 */
    WATCH_NAV,        /* 被采用的上位机导航指令，Chassis_Task 喂狗并处置超时 */
    WATCH_HOST,       /* 任一有效上位机帧，UartCom_Task 喂狗，仅指示 */
    WATCH_COUNT
} watchdog_source_t;

/* 状态由本模块持有；source 只接受上述有效枚举，不接受外部协议原值。
 * 仅在内核启动后由任务调用，内部以任务临界区保护时间戳和喂狗位图，调用方无需再加锁；ISR 不得调用。
 */

/* 任务主流程确认本源有效更新后直接调用，不在解析模块内隐藏喂狗。 */
void watchdog_control_feed(watchdog_source_t source);

/* 从未喂过或距上次喂狗 elapsed >= timeout 时返回 true，没有启动宽限期。
 * 返回电平量；需要“只处置一次”或上电未收到数据时保持默认状态的业务，由任务自己的有效位配合判断。
 */
bool watchdog_control_is_expired(watchdog_source_t source);

#endif
