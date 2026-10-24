#include "pid.h"

#include <stdbool.h>

static float constrain_abs(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_Init(PID_t *pid,
              float max_out,
              float integral_limit,
              float deadband,
              float kp,
              float ki,
              float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->max_out = max_out;
    pid->integral_limit = integral_limit;
    pid->deadband = deadband;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->expected = 0.0f;
    pid->feedback = 0.0f;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->proportional = 0.0f;
    pid->derivative = 0.0f;
    pid->saturated = false;
}

float PID_Calculate(PID_t *pid, float feedback, float reference)
{
    return PID_Calculate_Dt(pid, feedback, reference, 0.002f);
}

float PID_Calculate_Dt(PID_t *pid,
                       float feedback,
                       float reference,
                       float sample_period_s)
{
    float unsaturated_output;
    float integral_candidate;
    float error;
    float derivative;

    if (pid == 0 || sample_period_s <= 0.0f)
    {
        return 0.0f;
    }

    pid->expected = reference;
    pid->feedback = feedback;
    error = reference - feedback;
    if (error < pid->deadband && error > -pid->deadband)
    {
        error = 0.0f;
    }

    derivative = (error - pid->previous_error) / sample_period_s;
    integral_candidate = pid->integral + pid->ki * error * sample_period_s;
    integral_candidate = constrain_abs(integral_candidate, pid->integral_limit);
    unsaturated_output = pid->kp * error + integral_candidate + pid->kd * derivative;

    /* 误差继续推动输出向限幅外侧时，不再继续积分。 */
    if ((unsaturated_output > pid->max_out && error > 0.0f)
        || (unsaturated_output < -pid->max_out && error < 0.0f)) {
        unsaturated_output = pid->kp * error + pid->integral + pid->kd * derivative;
    } else {
        pid->integral = integral_candidate;
    }

    pid->error = error;
    pid->proportional = pid->kp * error;
    pid->derivative = pid->kd * derivative;
    pid->previous_error = error;
    pid->output = unsaturated_output;
    pid->saturated = false;
    if (pid->output > pid->max_out) {
        pid->output = pid->max_out;
        pid->saturated = true;
    } else if (pid->output < -pid->max_out) {
        pid->output = -pid->max_out;
        pid->saturated = true;
    }

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->proportional = 0.0f;
    pid->derivative = 0.0f;
    pid->saturated = false;
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
        return (float) ((int32_t) (value + 0.5f));
    }
    return (float) ((int32_t) (value - 0.5f));
}
