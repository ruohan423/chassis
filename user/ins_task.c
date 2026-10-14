//
// Created by mot on 2026/10/7.
//

#include <math.h>
#include <stdint.h>

#include "cmsis_os2.h"

#include "BMI088driver.h"
#include "vqf.h"

#define VQF_SAMPLE_PERIOD_S 0.005f
#define VQF_SAMPLE_PERIOD_MS 5U
#define RAD_TO_DEG 57.295779513f
#define ACCEL_GRAVITY_G 1.0f


typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float accel_earth_x;
    float accel_earth_y;
    float accel_earth_z;

    float linear_accel_x;
    float linear_accel_y;
    float linear_accel_z;

} vqf_output_t;

float gyro[3], accel[3], temp;
float quat6d[4];
volatile vqf_output_t vqf_output;
uint32_t vqf_next_sample_tick;

static void quat_to_euler_deg(const float q[4], float euler[3]);
static void quat_rotate_vector(const float q[4], const float v[3], float out[3]);


void INS_Task(void *argument) {

    uint32_t next_tick = osKernelGetTickCount();
    initVqf(VQF_SAMPLE_PERIOD_S, VQF_SAMPLE_PERIOD_S, 0.0f);

    for(;;)
    {

        BMI088_read(gyro, accel, &temp);
        updateGyr(gyro);
        updateAcc(accel);
        getQuat6D(quat6d);

        {
            float euler_deg[3];
            float accel_earth[3];

            quat_to_euler_deg(quat6d, euler_deg);
            quat_rotate_vector(quat6d, accel, accel_earth);

            vqf_output.roll_deg = euler_deg[0];
            vqf_output.pitch_deg = euler_deg[1];
            vqf_output.yaw_deg = euler_deg[2];

            vqf_output.accel_earth_x = accel_earth[0];
            vqf_output.accel_earth_y = accel_earth[1];
            vqf_output.accel_earth_z = accel_earth[2];

            vqf_output.linear_accel_x = accel_earth[0];
            vqf_output.linear_accel_y = accel_earth[1];
            vqf_output.linear_accel_z = accel_earth[2] - ACCEL_GRAVITY_G;
        }
        next_tick += VQF_SAMPLE_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}



static void quat_to_euler_deg(const float q[4], float euler[3])
{
    float sin_pitch;

    euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                      1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;

    sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }
    euler[1] = asinf(sin_pitch) * RAD_TO_DEG;

    euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
}

static void quat_rotate_vector(const float q[4], const float v[3], float out[3])
{
    float q_vec[3] = {q[1], q[2], q[3]};
    float t[3];
    float cross_qv[3];
    float cross_qt[3];

    cross_qv[0] = q_vec[1] * v[2] - q_vec[2] * v[1];
    cross_qv[1] = q_vec[2] * v[0] - q_vec[0] * v[2];
    cross_qv[2] = q_vec[0] * v[1] - q_vec[1] * v[0];

    t[0] = 2.0f * cross_qv[0];
    t[1] = 2.0f * cross_qv[1];
    t[2] = 2.0f * cross_qv[2];

    cross_qt[0] = q_vec[1] * t[2] - q_vec[2] * t[1];
    cross_qt[1] = q_vec[2] * t[0] - q_vec[0] * t[2];
    cross_qt[2] = q_vec[0] * t[1] - q_vec[1] * t[0];

    out[0] = v[0] + q[0] * t[0] + cross_qt[0];
    out[1] = v[1] + q[0] * t[1] + cross_qt[1];
    out[2] = v[2] + q[0] * t[2] + cross_qt[2];
}
