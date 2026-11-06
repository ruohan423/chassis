#include "board_led.h"

#include "stm32f4xx_hal.h"

#define BOARD_LED_GPIO_PORT GPIOH
#define BOARD_LED_RED_PIN GPIO_PIN_12
#define BOARD_LED_GREEN_PIN GPIO_PIN_11
#define BOARD_LED_BLUE_PIN GPIO_PIN_10
#define BOARD_LED_ALL_PINS (BOARD_LED_RED_PIN | BOARD_LED_GREEN_PIN | BOARD_LED_BLUE_PIN)
#define BOARD_LED_TOGGLE_PERIOD_MS 500U

static uint32_t board_led_toggle_tick[BOARD_LED_COUNT];

static uint16_t board_led_get_pin(board_led_t led)
{
    switch (led)
    {
        case BOARD_LED_RED:
            return BOARD_LED_RED_PIN;
        case BOARD_LED_GREEN:
            return BOARD_LED_GREEN_PIN;
        case BOARD_LED_BLUE:
            return BOARD_LED_BLUE_PIN;
        default:
            return 0U;
    }
}

void board_led_on(board_led_t led)
{
    board_led_set(led, true);
}

void board_led_off(board_led_t led)
{
    board_led_set(led, false);
}

void board_led_set(board_led_t led, bool on)
{
    const uint16_t pin = board_led_get_pin(led);
    if (pin == 0U){return;}

    HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 非阻塞限频翻转：同一 LED 两次翻转间隔不小于 BOARD_LED_TOGGLE_PERIOD_MS，周期任务可直接调用 */
void board_led_toggle(board_led_t led)
{
    const uint32_t now_tick = HAL_GetTick();

    if ((now_tick - board_led_toggle_tick[led]) < BOARD_LED_TOGGLE_PERIOD_MS){return;}

    board_led_toggle_tick[led] = now_tick;
    HAL_GPIO_TogglePin(BOARD_LED_GPIO_PORT, board_led_get_pin(led));
}

bool board_led_is_on(board_led_t led)
{
    const uint16_t pin = board_led_get_pin(led);
    return (pin != 0U) && (HAL_GPIO_ReadPin(BOARD_LED_GPIO_PORT, pin) == GPIO_PIN_SET);
}

void board_led_all_on(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, BOARD_LED_ALL_PINS, GPIO_PIN_SET);
}

void board_led_all_off(void)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT, BOARD_LED_ALL_PINS, GPIO_PIN_RESET);
}
