#include "buzzer.h"

#include "cmsis_os2.h"
#include "tim.h"

#define BUZZER_PULSE_PERIOD_MS 100U
#define BUZZER_ERROR_TICKS 10U
#define BUZZER_ON_DUTY 250U

static osTimerId_t buzzer_timer;
static volatile buzzer_tone_t buzzer_tone;
static volatile uint32_t buzzer_pulse_count;
static volatile uint32_t buzzer_pulse_index;
static volatile uint32_t buzzer_long_ticks;
static volatile bool buzzer_on;

static void buzzer_timer_callback(void *argument);

bool buzzer_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    if (HAL_TIM_Base_Start(&htim4) != HAL_OK){return false;}

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
    {
        (void)HAL_TIM_Base_Stop(&htim4);
        return false;
    }
    buzzer_on = false;
    buzzer_timer = osTimerNew(buzzer_timer_callback, osTimerPeriodic, NULL, NULL);
    if (buzzer_timer == NULL)
    {
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        (void)HAL_TIM_Base_Stop(&htim4);
        return false;
    }
    return true;
}

void buzzer_play(buzzer_tone_t tone)
{
    (void)osTimerStop(buzzer_timer);
    buzzer_tone = tone;
    buzzer_pulse_index = 0U;
    if (tone == BUZZER_TONE_STEP_DONE)
    {
        buzzer_pulse_count = 2U;
    }
    else if (tone == BUZZER_TONE_SUCCESS)
    {
        buzzer_pulse_count = 3U;
    }
    else
    {
        buzzer_pulse_count = 1U;
    }
    buzzer_long_ticks = BUZZER_ERROR_TICKS;
    buzzer_on = true;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_ON_DUTY);
    if (osTimerStart(buzzer_timer, BUZZER_PULSE_PERIOD_MS) != osOK)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
        buzzer_on = false;
    }
}

static void buzzer_timer_callback(void *argument)
{
    (void)argument;

    if (buzzer_tone == BUZZER_TONE_ERROR)
    {
        if (--buzzer_long_ticks == 0U)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
            buzzer_on = false;
            (void)osTimerStop(buzzer_timer);
        }
        return;
    }

    if (buzzer_on)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
        buzzer_on = false;
        buzzer_pulse_index++;
        if (buzzer_pulse_index >= buzzer_pulse_count)
        {
            (void)osTimerStop(buzzer_timer);
        }
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_ON_DUTY);
    buzzer_on = true;
}
