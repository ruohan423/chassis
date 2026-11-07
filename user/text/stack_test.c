#include "stack_test.h"

#if STACK_TEST_ENABLED
#define STACK_TEST_PERIOD_MS 1000U  /* 同一任务栈水位的采样间隔，单位 ms；首次调用立即采样。 */

volatile stack_test_result_t stack_test_results[STACK_TEST_SLOT_COUNT];

void StackTest_Record(uint32_t slot)
{
    if (slot >= STACK_TEST_SLOT_COUNT)
    {
        return;
    }

    const osThreadId_t thread_id = osThreadGetId();
    const uint32_t now_tick = osKernelGetTickCount();
    if (stack_test_results[slot].thread_id == thread_id
        && now_tick - stack_test_results[slot].sample_tick < (osKernelGetTickFreq() * STACK_TEST_PERIOD_MS + 999U) / 1000U)
    {
        return;
    }

    const char *name = osThreadGetName(thread_id);
    uint32_t index = 0U;
    while (index < STACK_TEST_NAME_SIZE - 1U && name[index] != '\0')
    {
        stack_test_results[slot].name[index] = name[index];
        index++;
    }
    stack_test_results[slot].name[index] = '\0';
    stack_test_results[slot].min_free_bytes = osThreadGetStackSpace(thread_id);
    stack_test_results[slot].sample_tick = now_tick;
    stack_test_results[slot].thread_id = thread_id;
}
#endif
