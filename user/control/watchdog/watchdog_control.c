#include "watchdog_control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "judge.h"
#include "watchdog.h"

#define REMOTE_WATCHDOG_TIMEOUT_MS 100U  /* 遥控链路超时，单位 ms；超时后 Remote_Task 清遥控数据并进入不锁存的停止态。 */
#define IMU_WATCHDOG_TIMEOUT_MS    100U  /* 姿态任务活动指示超时，单位 ms；不代表底层每次采样均有效。 */
#define NAV_WATCHDOG_TIMEOUT_MS    200U  /* 上位机导航指令超时，单位 ms；超时后 Chassis_Task 复位导航速度、目标朝向和行为请求。 */
#define HOST_WATCHDOG_TIMEOUT_MS   200U  /* 上位机链路指示超时，单位 ms；仅驱动指示灯，不代替导航和云台指令各自的失效保护。 */
/* 裁判链路超时直接取 judge.h 的 JUDGE_LINK_TIMEOUT_MS，与逐项新鲜度共用一个值。 */

_Static_assert(configTICK_RATE_HZ == 1000U, "软件看门狗的毫秒参数要求内核时钟为 1000 Hz");
_Static_assert(WATCH_COUNT <= 32U, "看门狗喂狗位图最多容纳 32 个源");

static watchdog_swdg_t software_watchdogs[WATCH_COUNT] = {
    [WATCH_REMOTE] = {.timeout_ms = REMOTE_WATCHDOG_TIMEOUT_MS},
    [WATCH_IMU] = {.timeout_ms = IMU_WATCHDOG_TIMEOUT_MS},
    [WATCH_JUDGE] = {.timeout_ms = JUDGE_LINK_TIMEOUT_MS},
    [WATCH_NAV] = {.timeout_ms = NAV_WATCHDOG_TIMEOUT_MS},
    [WATCH_HOST] = {.timeout_ms = HOST_WATCHDOG_TIMEOUT_MS},
};
static uint32_t fed_sources;  /* 上电后至少喂过一次的源，按 watchdog_source_t 编号置位。 */

void watchdog_control_feed(watchdog_source_t source)
{
    taskENTER_CRITICAL();
    watchdog_swdg_feed(&software_watchdogs[source]);
    fed_sources |= 1UL << source;
    taskEXIT_CRITICAL();
}

bool watchdog_control_is_expired(watchdog_source_t source)
{
    taskENTER_CRITICAL();
    const bool expired = (fed_sources & (1UL << source)) == 0U
        || watchdog_swdg_is_expired(&software_watchdogs[source]);
    taskEXIT_CRITICAL();
    return expired;
}
