#include "clock.h"
#include "main.h"

static ClockCore s_core;

static uint32_t CyclesPerUs(void)
{
    return SystemCoreClock / 1000000U;
}

void Clock_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_core.initialised = 0;
    ClockCore_Update(&s_core, DWT->CYCCNT);
}

uint64_t Clock_NowUs(void)
{
    ClockCore_Update(&s_core, DWT->CYCCNT);
    return ClockCore_Us(&s_core, CyclesPerUs());
}

uint32_t Clock_NowUs32(void)
{
    return (uint32_t)(Clock_NowUs() & 0xFFFFFFFFU);
}
