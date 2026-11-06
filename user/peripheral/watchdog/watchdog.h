#ifndef SENTRY_CHASSIS_WATCHDOG_H
#define SENTRY_CHASSIS_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct
{
    uint32_t last_feed_tick;
    uint32_t timeout_ms;
} watchdog_swdg_t;

void watchdog_swdg_init(watchdog_swdg_t *watchdog, uint32_t timeout_ms);
void watchdog_swdg_feed(watchdog_swdg_t *watchdog);
bool watchdog_swdg_is_expired(const watchdog_swdg_t *watchdog);

#if defined(HAL_IWDG_MODULE_ENABLED)
bool watchdog_iwdg_init(void);
bool watchdog_iwdg_refresh(void);
#endif

#if defined(HAL_WWDG_MODULE_ENABLED)
bool watchdog_wwdg_init(WWDG_HandleTypeDef *handle);
bool watchdog_wwdg_refresh(WWDG_HandleTypeDef *handle);
#endif

#endif
