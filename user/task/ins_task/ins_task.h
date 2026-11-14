//
// Created by mot on 2026/11/6.
//

#ifndef SENTRY_CHASSIS_INS_TASK_H
#define SENTRY_CHASSIS_INS_TASK_H

/* 姿态为 VQF 6D 解算的机体系（即 IMU 坐标系）相对地面系的欧拉角；角速度、加速度均在机体系下表示。 */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_x;  /* 机体系角速度，已扣除 VQF 零偏估计，单位 rad/s */
    float gyro_y;
    float gyro_z;
    float accel_x; /* 机体系比力（含重力），单位 m/s^2 */
    float accel_y;
    float accel_z;
} ins_output_t;

extern volatile ins_output_t ins_output;

void INS_Task(void *argument);

#endif //SENTRY_CHASSIS_INS_TASK_H
