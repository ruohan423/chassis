#ifndef SENTRY_CHASSIS_BUZZER_H
#define SENTRY_CHASSIS_BUZZER_H

#include <stdbool.h>

/* 蜂鸣器：TIM4_CH3 PWM，用 RTOS 周期定时器播放不同长度的脉冲串。
 * START 一声、STEP_DONE 两声、SUCCESS 三声、ERROR 一声长鸣。 */
typedef enum
{
    BUZZER_TONE_START = 0,
    BUZZER_TONE_STEP_DONE,
    BUZZER_TONE_SUCCESS,
    BUZZER_TONE_ERROR
} buzzer_tone_t;

bool buzzer_init(void);
/* 只在 buzzer_init 成功后调用 */
void buzzer_play(buzzer_tone_t tone);

#endif
