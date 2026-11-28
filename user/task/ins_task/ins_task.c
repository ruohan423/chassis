/* CubeMX _创建说明：保留或由用户创建姿态常驻线程。
 * 名称 ins_task，入口 INS_Task，优先级 osPriorityNormal，栈 2048 字节。
 * 若界面以 32 位字填写栈大小，填 512；当前采样任务要求内核时钟为 1000 Hz。
 * CAL_ON_STARTUP 非零时，还需按 calib_task.c 说明创建校准线程。
 * 本任务通过 Calib_TaskStart 通知已创建线程，不再动态创建校准线程。
 */
//
// 创建日期：2026/10/7。
//
#include "ins_task.h"
#include "can_commun.h"
#include "usb_commun.h"
#include "uart_commun.h"

#include <math.h>
#include <stdint.h>

#include "cmsis_os2.h"

#include "calib_task.h"
#include "bias_config.h"
#include "heater.h"
#include "watchdog_control.h"
#include "vqf.h"

#define RAD_TO_DEG 57.295779513f
static float imu_gyro[3];
static float imu_accel[3];
static float attitude_quaternion[4];
volatile ins_output_t ins_output;

static vqf_real_t gyro_bias[CAL_AXES];
static vqf_real_t gyro_bias_sigma;
static float imu_temperature_c;


static void quat_to_euler_deg(const float *q, float *euler);
static bool startup_ready(void);


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
        Calib_ReadImu(imu_gyro, imu_accel, &imu_temperature_c);
        watchdog_control_feed(WATCH_IMU);

        const uint32_t now_tick = osKernelGetTickCount();
        const uint32_t heater_elapsed_ticks = now_tick - last_heater_tick;
        if ((float)heater_elapsed_ticks * 0.001f >= HEATER_CONTROL_PERIOD_S)
        {
            (void)heater_update(imu_temperature_c, (float)heater_elapsed_ticks * 0.001f);
            last_heater_tick = now_tick;
        }

        updateGyr(imu_gyro);

        if (Calib_IsAccelReady())
        {
            updateAcc(imu_accel);
        }

        getQuat6D(attitude_quaternion);

        static float euler_degrees[3];
        static float current_gyro_bias[CAL_AXES];

        /* IMU 坐标系即机体坐标系：角速度扣除 VQF 当前零偏估计，加速度为机体系比力（含重力）。 */
        quat_to_euler_deg(attitude_quaternion, euler_degrees);
        (void)getBiasEstimate(current_gyro_bias);
        const float gyro_z = imu_gyro[2] - current_gyro_bias[2];
        const float yaw_rad = euler_degrees[2] * 0.01745329252f;

        const int32_t previous_lock = osKernelLock();
        ins_output.roll_deg = euler_degrees[0];
        ins_output.pitch_deg = euler_degrees[1];
        ins_output.yaw_deg = euler_degrees[2];
        ins_output.gyro_x = imu_gyro[0] - current_gyro_bias[0];
        ins_output.gyro_y = imu_gyro[1] - current_gyro_bias[1];
        ins_output.gyro_z = gyro_z;
        ins_output.accel_x = imu_accel[0];
        ins_output.accel_y = imu_accel[1];
        ins_output.accel_z = imu_accel[2];
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        usb_commun_update_imu(yaw_rad, gyro_z);
        uart_commun_update_imu(yaw_rad, gyro_z);
        can_commun_update_imu(yaw_rad);

        next_tick += 1U;
        osDelayUntil(next_tick);
    }
}

static bool startup_ready(void)
{
#if CAL_ON_STARTUP != 0U
    if (Calib_TaskStart())
    {
        while (!Calib_IsReady() && !Calib_IsFailed()) osDelay(10U);
        if (Calib_IsReady())
        {
            Calib_GetBias(gyro_bias, &gyro_bias_sigma);
            return true;
        }
    }
    /* 校准未成功时回退固定零偏，继续姿态解算。 */
#endif
    gyro_bias[0] = GYR_BIAS_X;
    gyro_bias[1] = GYR_BIAS_Y;
    gyro_bias[2] = GYR_BIAS_Z;
    gyro_bias_sigma = fmaxf(GYR_BIAS_SIGMA, GYRO_SIGMA_MIN);
#if CAL_ON_STARTUP != 0U
    /* 加热启动失败时保持零输出，不阻止固定零偏下的姿态解算。 */
    (void)heater_init();
    return true;
#else
    return heater_init();
#endif
}

static void quat_to_euler_deg(const float *q, float *euler)
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
