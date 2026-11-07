#include "watchdog.h"

#include "cmsis_os2.h"

#if defined(HAL_IWDG_MODULE_ENABLED)
static IWDG_HandleTypeDef iwdg_handle;
static bool iwdg_started;
#endif

/* 内核 tick 为 1kHz，超时直接按毫秒计 */
void watchdog_swdg_init(watchdog_swdg_t *watchdog, uint32_t timeout_ms)
{
    watchdog->last_feed_tick = osKernelGetTickCount();
    watchdog->timeout_ms = timeout_ms;
}

void watchdog_swdg_feed(watchdog_swdg_t *watchdog)
{
    watchdog->last_feed_tick = osKernelGetTickCount();
}

bool watchdog_swdg_is_expired(const watchdog_swdg_t *watchdog)
{
    return (osKernelGetTickCount() - watchdog->last_feed_tick) > watchdog->timeout_ms;
}

#if defined(HAL_IWDG_MODULE_ENABLED)
bool watchdog_iwdg_init(void)
{
    iwdg_handle.Instance = IWDG;
    iwdg_handle.Init.Prescaler = IWDG_PRESCALER_64;
    iwdg_handle.Init.Reload = 499U;
    iwdg_started = HAL_IWDG_Init(&iwdg_handle) == HAL_OK;
    return iwdg_started;
}

bool watchdog_iwdg_refresh(void)
{
    return iwdg_started && HAL_IWDG_Refresh(&iwdg_handle) == HAL_OK;
}
#endif

#if defined(HAL_WWDG_MODULE_ENABLED)
bool watchdog_wwdg_init(WWDG_HandleTypeDef *handle)
{
    return handle != NULL && HAL_WWDG_Init(handle) == HAL_OK;
}

bool watchdog_wwdg_refresh(WWDG_HandleTypeDef *handle)
{
    return handle != NULL && HAL_WWDG_Refresh(handle) == HAL_OK;
}
#endif
