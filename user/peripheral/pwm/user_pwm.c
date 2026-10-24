#include "user_pwm.h"

#include <math.h>

#include "bias_config.h"
#include "cmsis_os2.h"
#include "pid.h"

static PID_t heater_pid;
static bool heater_fault;

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle,
                 uint32_t channel,
                 uint16_t value)
{
    uint32_t arr;

    if (tim_pwmHandle == NULL || tim_pwmHandle->Instance == NULL)
    {
        return;
    }

    arr = tim_pwmHandle->Instance->ARR;
    if (value > arr)
    {
        value = (uint16_t) arr;
    }

    switch (channel)
    {
        case TIM_CHANNEL_1:
            tim_pwmHandle->Instance->CCR1 = value;
            break;
        case TIM_CHANNEL_2:
            tim_pwmHandle->Instance->CCR2 = value;
            break;
        case TIM_CHANNEL_3:
            tim_pwmHandle->Instance->CCR3 = value;
            break;
        case TIM_CHANNEL_4:
            tim_pwmHandle->Instance->CCR4 = value;
            break;
        default:
            break;
    }
}

bool Pwm_HeaterInit(void)
{
    PID_Init(&heater_pid, HEATER_OUT_MAX,
             HEATER_OUT_MAX * HEATER_INTEGRAL_LIMIT_RATIO, 0.0f,
             HEATER_KP, HEATER_KI, HEATER_KD);
    heater_fault = false;
    if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK)
    {
        heater_fault = true;
        return false;
    }
    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, 0U);
    return true;
}

bool Pwm_HeaterUpdate(float temperature_c, float sample_period_s)
{
    float output;
    float compare_value;

    if (!isfinite(temperature_c) || temperature_c < -40.0f || temperature_c > 125.0f
        || temperature_c >= TEMP_LIMIT_C || !isfinite(sample_period_s) || sample_period_s <= 0.0f)
    {
        heater_fault = true;
        PID_Reset(&heater_pid);
        Pwm_HeaterStop();
        return false;
    }
    if (heater_fault) return false;
    if (temperature_c > TEMP_TARGET_C)
    {
        Pwm_HeaterStop();
        return true;
    }

    output = PID_Calculate_Dt(&heater_pid, temperature_c, TEMP_TARGET_C, sample_period_s);
    output = float_constrain(output, 0.0f, HEATER_OUT_MAX);
    compare_value = output / HEATER_OUT_MAX * (float)htim10.Instance->ARR;
    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, (uint16_t)float_rounding(compare_value));
    return true;
}

void Pwm_HeaterStop(void)
{
    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, 0U);
}

bool Pwm_BuzzerInit(void)
{
    if (HAL_TIM_Base_Start(&htim4) != HAL_OK
        || HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
    {
        return false;
    }
    TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 0U);
    return true;
}

void Pwm_BuzzerPlay(pwm_buzzer_result_t result)
{
    if (result == PWM_BUZZER_CALIBRATION_SUCCESS)
    {
        TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 250U);
        osDelay(300U);
    }
    else
    {
        for (uint32_t pulse = 0U; pulse < 2U; ++pulse)
        {
            TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 250U);
            osDelay(100U);
            TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 0U);
            osDelay(100U);
        }
    }
    TIM_Set_PWM(&htim4, TIM_CHANNEL_3, 0U);
}
