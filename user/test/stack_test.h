#ifndef SENTRY_CHASSIS_STACK_TEST_H
#define SENTRY_CHASSIS_STACK_TEST_H

#include "cmsis_os2.h"

#ifndef STACK_TEST_ENABLED
#define STACK_TEST_ENABLED 0U  /* 栈采样开关：0 关闭，1 启用；仅上板测量时开启。 */
#endif
#define STACK_TEST_SLOT_COUNT 8U  /* 采样槽位数，无单位；每个槽位只能由一个任务写入，索引范围为 0～7。 */
#define STACK_TEST_NAME_SIZE 16U  /* 任务名存储长度，单位字节，包含结尾空字符；超长名称截断显示。 */

#if STACK_TEST_ENABLED
typedef enum
{
    STACK_TEST_JUDGE = 0,
    STACK_TEST_POWER,
    STACK_TEST_CHASSIS
} stack_test_slot_t;

typedef struct
{
    osThreadId_t thread_id;
    char name[STACK_TEST_NAME_SIZE];
    uint32_t min_free_bytes;  /* 任务启动以来的最小剩余栈，单位字节；不是当前瞬时空闲量。 */
    uint32_t sample_tick;     /* 最近一次采样的内核 tick；thread_id 为 NULL 表示尚未采样。 */
} stack_test_result_t;

/* 仅用于调试器观察；不同槽位独立更新，不作为任务间的一致性快照使用。 */
extern volatile stack_test_result_t stack_test_results[STACK_TEST_SLOT_COUNT];

/* 仅由被测任务调用，不用于 ISR；可放在循环中，内部限制为每秒采样一次。 */
void StackTest_Record(uint32_t slot);
#endif

#endif
