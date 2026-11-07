/* CubeMX 创建说明：保留或由用户创建姿态常驻线程。
 * 名称 ins_task，入口 INS_Task，优先级 osPriorityNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；当前采样任务要求内核时钟为 1000 Hz。
 * CAL_ON_STARTUP 非零时，还需按 calib_task.c 说明创建校准线程。
 * 本任务通过 Calib_TaskStart 通知已创建线程，不再动态创建校准线程。
 */
//
// 创建日期：2026/10/7。
//
#include "ins_task.h"

#include <math.h>
#include <stdint.h>

#include "cmsis_os2.h"

#include "calib_task.h"
#include "bias_config.h"
#include "heater.h"
#include "watch_task.h"
#include "vqf.h"

#define RAD_TO_DEG 57.295779513f
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float accel_earth_x;
    float accel_earth_y;
    float accel_earth_z;

} vqf_output_t;

float gyro[3], accel[3];
float quat6d[4];
volatile vqf_output_t vqf_output;

static vqf_real_t gyro_bias[CAL_AXES];
static vqf_real_t gyro_bias_sigma;
float temperature_c;


static void quat_to_euler_deg(const float q[4], float euler[3]);
static void quat_rotate_vector(const float q[4], const float v[3], float out[3]);
static bool startup_ready(void);
#if CAL_ON_STARTUP == 0U
static void load_bias(void);
#endif


void INS_Task(void *argument)
{
    uint32_t next_tick;
    uint32_t last_heater_tick;
    (void) argument;

    if (!startup_ready())
    {
        for (;;)
        {
            osDelay(1000U);
        }
    }
    initVqf(VQF_GYR_DT, VQF_ACC_DT, 0.0f);
    setBiasEstimate(gyro_bias, gyro_bias_sigma);
    next_tick = osKernelGetTickCount();
    last_heater_tick = next_tick;

    for (;;)
    {
        Calib_ReadImu(gyro, accel, &temperature_c);
        swdg_feed_imu();

        const uint32_t now_tick = osKernelGetTickCount();
        const uint32_t heater_elapsed_ticks = now_tick - last_heater_tick;
        if ((float)heater_elapsed_ticks * 0.001f >= HEATER_CONTROL_PERIOD_S)
        {
            (void)heater_update(temperature_c, (float)heater_elapsed_ticks * 0.001f);
            last_heater_tick = now_tick;
        }

        updateGyr(gyro);

        if (Calib_IsAccelReady())
        {
            updateAcc(accel);
        }

        getQuat6D(quat6d);

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

        next_tick += 1U;
        osDelayUntil(next_tick);
    }
}

static bool startup_ready(void)
{
#if CAL_ON_STARTUP != 0U
    if (!Calib_TaskStart()) return false;
    while (!Calib_IsReady() && !Calib_IsFailed()) osDelay(10U);
    if (!Calib_IsReady()) return false;
    Calib_GetBias(gyro_bias, &gyro_bias_sigma);
    return true;
#else
    load_bias();
    return heater_init();
#endif
}

#if CAL_ON_STARTUP == 0U
static void load_bias(void)
{
    gyro_bias[0] = GYR_BIAS_X;
    gyro_bias[1] = GYR_BIAS_Y;
    gyro_bias[2] = GYR_BIAS_Z;
    gyro_bias_sigma = fmaxf(GYR_BIAS_SIGMA, GYRO_SIGMA_MIN);
}
#endif

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
