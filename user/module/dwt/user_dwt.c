#include "user_dwt.h"

#include "stm32f4xx.h"

bool dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U;
}

uint32_t dwt_get_cycles(void)
{
    return DWT->CYCCNT;
}

uint32_t dwt_get_elapsed_cycles(uint32_t start_cycles)
{
    return dwt_get_cycles() - start_cycles;
}

uint32_t dwt_cycles_to_us(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

bool dwt_delay_us(uint32_t delay_us)
{
    const uint64_t delay_cycles = ((uint64_t)SystemCoreClock * delay_us + 999999ULL) / 1000000ULL;
    const uint32_t start_cycles = dwt_get_cycles();

    if (delay_cycles > UINT32_MAX){return false;}

    while (dwt_get_elapsed_cycles(start_cycles) < (uint32_t)delay_cycles) {

    }

    return true;
}
