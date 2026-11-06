#ifndef SENTRY_CHASSIS_BOARD_LED_H
#define SENTRY_CHASSIS_BOARD_LED_H

#include <stdbool.h>

typedef enum
{
    BOARD_LED_RED = 0,
    BOARD_LED_GREEN,
    BOARD_LED_BLUE,
    BOARD_LED_COUNT
} board_led_t;

void board_led_on(board_led_t led);
void board_led_off(board_led_t led);
void board_led_set(board_led_t led, bool on);
void board_led_toggle(board_led_t led);
bool board_led_is_on(board_led_t led);
void board_led_all_on(void);
void board_led_all_off(void);

#endif
