#ifndef VQF_PID_H
#define VQF_PID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float max_out;
    float integral_limit;
    float deadband;
    float kp;
    float ki;
    float kd;
    float expected;
    float feedback;
    float integral;
    float previous_error;
    float output;
    float error;
    float proportional;
    float derivative;
    bool saturated;
} PID_t;

void PID_Init(PID_t *pid,
              float max_out,
              float integral_limit,
              float deadband,
              float kp,
              float ki,
              float kd);

float PID_Calculate(PID_t *pid, float feedback, float reference);
float PID_Calculate_Dt(PID_t *pid,
                       float feedback,
                       float reference,
                       float sample_period_s);
void PID_Reset(PID_t *pid);

float float_constrain(float value, float min_value, float max_value);
float float_rounding(float value);

#endif /* VQF_PID_H */
