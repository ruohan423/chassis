#include "calib_task.h"

#include <math.h>
#include <stdint.h>

#include "BMI088driver.h"
#include "bias_config.h"
#include "calibration_session.h"
#include "cmsis_os2.h"
#include "buzzer.h"
#include "heater.h"

static volatile bool calibration_ready;
static volatile bool calibration_failed;
static float calibrated_bias[CAL_AXES];
static float calibrated_sigma;
static float sensor_temp_c;
static cal_session_cfg_t calibration_config;
static cal_session_t calibration_session;

static const osThreadAttr_t calib_task_attributes = {
    .name = "calib_task",
    .stack_size = 512U * 4U,
    .priority = osPriorityAboveNormal,
};

static void Calib_Task(void *argument);
static bool run_gyro_cal(void);

/* 只在 INS 任务启动阶段调用一次；校准任务退出后 INS 才开始读 IMU，两者不并发 */
bool Calib_TaskStart(void)
{
    calibration_ready = false;
    calibration_failed = false;
    if (osThreadNew(Calib_Task, NULL, &calib_task_attributes) == NULL)
    {
        calibration_failed = true;
        return false;
    }
    return true;
}

static void Calib_Task(void *argument)
{
    (void)argument;
    if (!heater_init() || !buzzer_init())
    {
        calibration_failed = true;
        osThreadExit();
    }

    if (run_gyro_cal())
    {
        buzzer_play(BUZZER_TONE_SUCCESS);
    }
    else
    {
        calibration_failed = true;
        buzzer_play(BUZZER_TONE_ERROR);
        heater_stop();
    }

    osThreadExit();
}

bool Calib_IsReady(void)
{
    return calibration_ready;
}

bool Calib_IsFailed(void)
{
    return calibration_failed;
}

void Calib_ReadImu(float gyro[CAL_AXES], float accel[CAL_AXES], float *temperature_c)
{
    BMI088_read(gyro, accel, temperature_c);
}

bool Calib_IsAccelReady(void)
{
    return BMI088_accel_data_ready() != 0U;
}

/* 仅在 Calib_IsReady 为真后调用 */
void Calib_GetBias(float bias[CAL_AXES], float *sigma)
{
    for (uint32_t axis = 0U; axis < CAL_AXES; ++axis) bias[axis] = calibrated_bias[axis];
    *sigma = calibrated_sigma;
}

static bool run_gyro_cal(void)
{
    float gyro[CAL_AXES];
    float accel[CAL_AXES];
    uint32_t next_tick = osKernelGetTickCount();
    uint32_t last_heater_tick = next_tick;
    bool heater_update_due = true;

    cal_session_default(&calibration_config);
    cal_session_init(&calibration_session, &calibration_config);
    cal_session_start(&calibration_session);
    buzzer_play(BUZZER_TONE_START);

    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();
        const cal_session_state_t previous_state = cal_session_state(&calibration_session);
        cal_status_t status;

        Calib_ReadImu(gyro, accel, &sensor_temp_c);
        if (heater_update_due || (float)(now_tick - last_heater_tick) * 0.001f >= HEATER_CONTROL_PERIOD_S)
        {
            /* 首次更新间隔为 0，至少按 1ms 计算避免 PID 微分除零 */
            const uint32_t elapsed_ticks = now_tick - last_heater_tick;
            if (!heater_update(sensor_temp_c, (float)(elapsed_ticks ? elapsed_ticks : 1U) * 0.001f)) return false;
            last_heater_tick = now_tick;
            heater_update_due = false;
        }

        status = cal_session_push(&calibration_session, now_tick, gyro, accel, Calib_IsAccelReady(), sensor_temp_c);
        if (previous_state == CAL_SESSION_WARMUP && cal_session_state(&calibration_session) == CAL_SESSION_SAMPLE)
        {
            buzzer_play(BUZZER_TONE_STEP_DONE);
        }
        if (status == CAL_OK)
        {
            const gyro_bias_t *result = cal_session_result(&calibration_session);
            for (uint32_t axis = 0U; axis < CAL_AXES; ++axis) calibrated_bias[axis] = result->bias[axis];
            calibrated_sigma = fmaxf(result->sigma, GYRO_SIGMA_MIN);
            calibration_ready = true;
            return true;
        }
        if (status >= CAL_GYR_MOTION) return false;

        next_tick += 1U;
        osDelayUntil(next_tick);
    }
}
