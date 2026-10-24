#ifndef VQF_PWM_H
#define VQF_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle,
                 uint32_t channel,
                 uint16_t value);

typedef enum
{
    PWM_BUZZER_CALIBRATION_SUCCESS = 0,
    PWM_BUZZER_CALIBRATION_FAILURE
} pwm_buzzer_result_t;

bool Pwm_HeaterInit(void);
bool Pwm_HeaterUpdate(float temperature_c, float sample_period_s);
void Pwm_HeaterStop(void);
bool Pwm_BuzzerInit(void);
void Pwm_BuzzerPlay(pwm_buzzer_result_t result);

#endif /* VQF_PWM_H */
