#include "pid.h"

#include <math.h>
#include <stdint.h>

static float constrain_abs(float value, float limit)
{
    limit = fabsf(limit);
    return float_constrain(value, -limit, limit);
}

void pid_position_init(pid_position_t *pid, float kp, float ki, float kd, float integral_limit,float sample_period_s)
{
    *pid = (pid_position_t){.kp = kp, .ki = ki, .kd = kd,
                            .integral_limit = fabsf(integral_limit),
                            .sample_period_s = sample_period_s};
}

float pid_position_calculate(pid_position_t *pid, float feedback, float reference)
{
    pid->error = reference - feedback;
    pid->integral = constrain_abs(pid->integral + pid->error * pid->sample_period_s,pid->integral_limit);
    pid->output = pid->kp * pid->error + pid->ki * pid->integral
        + pid->kd * (pid->error - pid->previous_error) / pid->sample_period_s;
    pid->previous_error = pid->error;
    return pid->output;
}

void pid_ffb_init(pid_ffb_t *pid, float kp, float ki, float kd, float kf, float integral_limit,float sample_period_s)
{
    *pid = (pid_ffb_t){.kp = kp, .ki = ki, .kd = kd, .kf = kf,
                       .integral_limit = fabsf(integral_limit),
                       .sample_period_s = sample_period_s};
}

float pid_ffb_calculate(pid_ffb_t *pid, float feedback, float reference)
{
    pid->error = reference - feedback;
    pid->integral = constrain_abs(pid->integral + pid->error * pid->sample_period_s,pid->integral_limit);
    pid->output = pid->kp * pid->error + pid->ki * pid->integral
        + pid->kd * (pid->error - pid->previous_error) / pid->sample_period_s
        + pid->kf * (reference - pid->previous_reference) / pid->sample_period_s;
    pid->previous_error = pid->error;
    pid->previous_reference = reference;
    return pid->output;
}

void pid_dom_init(pid_dom_t *pid, float kp, float ki, float kd, float integral_limit,
                  float sample_period_s)
{
    *pid = (pid_dom_t){.kp = kp, .ki = ki, .kd = kd,
                       .integral_limit = fabsf(integral_limit),
                       .sample_period_s = sample_period_s};
}

float pid_dom_calculate(pid_dom_t *pid, float feedback, float reference)
{
    pid->error = reference - feedback;
    pid->integral = constrain_abs(pid->integral + pid->error * pid->sample_period_s,pid->integral_limit);
    pid->output = pid->kp * pid->error + pid->ki * pid->integral
        - pid->kd * (feedback - pid->previous_feedback) / pid->sample_period_s;
    pid->previous_feedback = feedback;
    return pid->output;
}

void pid_increment_init(pid_increment_t *pid, float kp, float ki, float kd,
                        float sample_period_s)
{
    *pid = (pid_increment_t){.kp = kp, .ki = ki, .kd = kd,
                             .sample_period_s = sample_period_s};
}

float pid_increment_calculate(pid_increment_t *pid, float feedback, float reference)
{
    pid->error = reference - feedback;
    pid->increment = pid->kp * (pid->error - pid->previous_error)
        + pid->ki * pid->error * pid->sample_period_s
        + pid->kd * (pid->error - 2.0f * pid->previous_error
                     + pid->previous_previous_error) / pid->sample_period_s;
    pid->previous_previous_error = pid->previous_error;
    pid->previous_error = pid->error;
    return pid->increment;
}

float float_constrain(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

float float_rounding(float value)
{
    if (value >= 0.0f)
    {
        return (float)((int32_t)(value + 0.5f));
    }
    return (float)((int32_t)(value - 0.5f));
}
