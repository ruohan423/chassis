#include "PID.h"

static float clampf_(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void PID_init(PID *p, float kp, float ki, float kd, float integralMax, float outputMax, float alpha, float dt)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integralMax = integralMax; p->outputMax = outputMax;
    p->alpha = alpha; p->dt = dt;
    PID_reset(p);
}

void PID_reset(PID *p)
{
    p->integral = 0.0f; p->lastError = 0.0f; p->dFilt = 0.0f; p->first = 1;
}

float PID_run(PID *p, float set, float fb)
{
    float err = set - fb;
    p->integral = clampf_(p->integral + err * p->dt, -p->integralMax, p->integralMax);
    float d = p->first ? 0.0f : (err - p->lastError) / p->dt;
    p->first = 0;
    p->lastError = err;
    p->dFilt = p->alpha * d + (1.0f - p->alpha) * p->dFilt;
    return clampf_(p->kp * err + p->ki * p->integral + p->kd * p->dFilt, -p->outputMax, p->outputMax);
}
