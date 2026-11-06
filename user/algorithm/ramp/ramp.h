
#ifndef SENTRY_CHASSIS_RAMP_H
#define SENTRY_CHASSIS_RAMP_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t start_tick;
    uint32_t duration_ticks;
    bool started;
} ramp_t;

/* 从首次调用起由 0 匀速上升到 1，时间源由调用方传入；时长为 0 时立即完成。
 * 初始化时设置 duration_ticks，其余字段置零；计时单位须与 now_tick 一致。 */
float ramp_slope(ramp_t *ramp, uint32_t now_tick);
void ramp_reset(ramp_t *ramp);

/* step 为每次调用允许的非负变化量，到达目标时不再越过目标。 */
int16_t ramp_step(int16_t target, int16_t current, uint16_t step);
float ramp_step_float(float target, float current, float step);

/* 返回 current 向 target 变化、变化率不超过 rate_limit [单位/秒] 后的值。
 * rate_limit 与 target/current 单位一致，delta_s 为本次调用距上次的秒数。 */
float ramp_limit(float target, float current, float rate_limit, float delta_s);

#endif
