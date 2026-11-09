//
// Created by mot on 2026/11/6.
//

#ifndef SENTRY_CHASSIS_INS_TASK_H
#define SENTRY_CHASSIS_INS_TASK_H

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float accel_earth_x;
    float accel_earth_y;
    float accel_earth_z;
} ins_output_t;

extern volatile ins_output_t ins_output;

void INS_Task(void *argument);

#endif //SENTRY_CHASSIS_INS_TASK_H
