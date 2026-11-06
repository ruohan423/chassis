#include "ramp.h"

float ramp_slope(ramp_t *ramp, uint32_t now_tick)
{
    if (!ramp->started)
    {
        ramp->start_tick = now_tick;
        ramp->started = true;
    }

    const uint32_t elapsed_ticks = now_tick - ramp->start_tick;
    if (elapsed_ticks >= ramp->duration_ticks)
    {
        return 1.0f;
    }

    return (float)elapsed_ticks / (float)ramp->duration_ticks;
}

void ramp_reset(ramp_t *ramp)
{
    ramp->started = false;
}

int16_t ramp_step(int16_t target, int16_t current, uint16_t step)
{
    const int32_t difference = (int32_t)target - current;

    if (difference > (int32_t)step)
    {
        return (int16_t)((int32_t)current + step);
    }
    if (difference < -(int32_t)step)
    {
        return (int16_t)((int32_t)current - step);
    }

    return target;
}

float ramp_step_float(float target, float current, float step)
{
    if ((target - current) > step)
    {
        return current + step;
    }
    if ((current - target) > step)
    {
        return current - step;
    }

    return target;
}

float ramp_limit(float target, float current, float rate_limit, float delta_s)
{
    return ramp_step_float(target, current, rate_limit * delta_s);
}
