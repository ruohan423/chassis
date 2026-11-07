#ifndef USER_DWT_H
#define USER_DWT_H

#include <stdbool.h>
#include <stdint.h>

bool dwt_init(void);
uint32_t dwt_get_cycles(void);
uint32_t dwt_get_elapsed_cycles(uint32_t start_cycles);
uint32_t dwt_cycles_to_us(uint32_t cycles);
bool dwt_delay_us(uint32_t delay_us);

/*
 * FreeRTOS 注意事项：
 * 1. DWT->CYCCNT 是全局自由运行计数器，任务读取时不要清零或重复初始化。
 * 2. 应在启动调度器前初始化 DWT；调度器运行后，任务间的周期差也会包含其他任务和中断占用的时间。
 * 3. dwt_delay_us() 是忙等，会占用 CPU 并阻止当前任务让出处理器，只用于很短的硬件等待；任务延时应使用 osDelay() 或 osDelayUntil()。
 * 4. 微秒换算使用 SystemCoreClock；系统时钟变化后需确保该变量已更新。
 */

#endif
