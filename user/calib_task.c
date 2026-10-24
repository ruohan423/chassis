#include "calib_task.h"

#include <math.h>
#include <stdint.h>

#include "BMI088driver.h"
#include "bias_config.h"
#include "calibration_session.h"
#include "cmsis_os2.h"
#include "user_pwm.h"

static osMutexId_t imu_mutex;
static volatile bool calibration_ready;
static volatile bool calibration_failed;
static float calibrated_bias[CAL_AXES];
static float calibrated_sigma;
static float sensor_temp_c;
static osThreadId_t calib_task_id;
static cal_session_cfg_t calibration_config;
static cal_session_t calibration_session;

static const osThreadAttr_t calib_task_attributes = {
    .name = "calib_task",
    .stack_size = 512U * 4U,
    .priority = osPriorityAboveNormal,
};

static void Calib_Task(void *argument);
static bool run_gyro_cal(void);

bool Calib_TaskStart(void)
{
    if (CAL_ON_STARTUP == 0U || calib_task_id != NULL) return false;

    calibration_ready = false;
    calibration_failed = false;
    if (imu_mutex == NULL)
    {
        imu_mutex = osMutexNew(NULL);
        if (imu_mutex == NULL)
        {
            calibration_failed = true;
            return false;
        }
    }

    calib_task_id = osThreadNew(Calib_Task, NULL, &calib_task_attributes);
    if (calib_task_id == NULL)
    {
        calibration_failed = true;
        return false;
    }
    return true;
}
static void Calib_Task(void *argument)
{
    (void)argument;
    if (!Pwm_HeaterInit() || !Pwm_BuzzerInit())
    {
        calibration_failed = true;
        osThreadExit();
    }

    if (run_gyro_cal())
    {
        Pwm_BuzzerPlay(PWM_BUZZER_CALIBRATION_SUCCESS);
    }
    else
    {
        calibration_failed = true;
        Pwm_BuzzerPlay(PWM_BUZZER_CALIBRATION_FAILURE);
        Pwm_HeaterStop();
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

bool Calib_ReadImu(float gyro[CAL_AXES], float accel[CAL_AXES], float *temperature_c)
{
    if (!gyro || !accel || !temperature_c) return false;
    if (imu_mutex && osMutexAcquire(imu_mutex, osWaitForever) != osOK) return false;

    BMI088_read(gyro, accel, temperature_c);
    if (imu_mutex) (void)osMutexRelease(imu_mutex);
    return true;
}

bool Calib_IsAccelReady(void)
{
    uint8_t ready;

    if (imu_mutex && osMutexAcquire(imu_mutex, osWaitForever) != osOK) return false;
    ready = BMI088_accel_data_ready();
    if (imu_mutex) (void)osMutexRelease(imu_mutex);
    return ready != 0U;
}

bool Calib_GetBias(float bias[CAL_AXES], float *sigma)
{
    if (!bias || !sigma || !calibration_ready) return false;
    for (uint32_t axis = 0U; axis < CAL_AXES; ++axis) bias[axis] = calibrated_bias[axis];
    *sigma = calibrated_sigma;
    return true;
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
    if (cal_session_state(&calibration_session) != CAL_SESSION_IDLE) return false;
    if (!cal_session_start(&calibration_session)) return false;

    for (;;)
    {
        const uint32_t now_tick = osKernelGetTickCount();
        cal_status_t status;

        if (!Calib_ReadImu(gyro, accel, &sensor_temp_c)) return false;
        if (heater_update_due || (float)(now_tick - last_heater_tick) * 0.001f >= HEATER_CONTROL_PERIOD_S)
        {
            const uint32_t elapsed_ticks = now_tick - last_heater_tick;
            if (!Pwm_HeaterUpdate(sensor_temp_c,
                                  (float)(elapsed_ticks ? elapsed_ticks : 1U) * 0.001f))
            {
                return false;
            }
            last_heater_tick = now_tick;
            heater_update_due = false;
        }

        status = cal_session_push(&calibration_session, now_tick, gyro, accel,
                                  Calib_IsAccelReady(), sensor_temp_c);
        if (status == CAL_OK)
        {
            const gyro_bias_t *result = cal_session_result(&calibration_session);
            if (!result) return false;
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
