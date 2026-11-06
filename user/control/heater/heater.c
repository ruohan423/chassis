#include "heater.h"

#include <math.h>

#include "bias_config.h"
#include "tim.h"

typedef struct
{
    float max_out;
    float integral_limit;
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
} heater_pid_t;

static heater_pid_t heater_pid;

static float heater_pid_calculate(float temperature_c, float sample_period_s);

bool heater_init(void)
{
    heater_pid = (heater_pid_t){
        .max_out = HEATER_OUT_MAX,
        .integral_limit = HEATER_OUT_MAX * HEATER_INTEGRAL_LIMIT_RATIO,
        .kp = HEATER_KP,
        .ki = HEATER_KI,
        .kd = HEATER_KD,
    };

    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0U);
    return HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) == HAL_OK;
}

bool heater_update(float temperature_c, float sample_period_s)
{
    /* 超过安全上限：复位积分并关闭加热 */
    if (temperature_c >= TEMP_LIMIT_C)
    {
        heater_pid.integral = 0.0f;
        heater_pid.previous_error = 0.0f;
        heater_stop();
        return false;
    }

    if (temperature_c > TEMP_TARGET_C)
    {
        /* 加热器不能主动降温，超过目标温度后关闭 PWM 自然冷却。 */
        heater_stop();
        return true;
    }

    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,
                         (uint16_t)(heater_pid_calculate(temperature_c, sample_period_s) / HEATER_OUT_MAX
                                    * (float)__HAL_TIM_GET_AUTORELOAD(&htim10) + 0.5f));
    return true;
}

void heater_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0U);
}

static float heater_pid_calculate(float temperature_c, float sample_period_s)
{
    float unsaturated_output;
    float integral_candidate;
    float error;
    float derivative;

    error = TEMP_TARGET_C - temperature_c;
    derivative = (error - heater_pid.previous_error) / sample_period_s;
    integral_candidate = heater_pid.integral + heater_pid.ki * error * sample_period_s;
    integral_candidate = fminf(fmaxf(integral_candidate, -heater_pid.integral_limit),
                               heater_pid.integral_limit);
    unsaturated_output = heater_pid.kp * error + integral_candidate + heater_pid.kd * derivative;

    /* 输出已到加热上限且误差仍要求升温时，停止继续积分。 */
    if (unsaturated_output > heater_pid.max_out && error > 0.0f)
    {
        unsaturated_output = heater_pid.kp * error + heater_pid.integral
            + heater_pid.kd * derivative;
    }
    else
    {
        heater_pid.integral = integral_candidate;
    }

    heater_pid.previous_error = error;
    return fminf(fmaxf(unsaturated_output, 0.0f), heater_pid.max_out);
}
