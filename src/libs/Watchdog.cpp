#include "Watchdog.h"

#include <lpc17xx_wdt.h>

#include <mri.h>

Watchdog::Watchdog(uint32_t timeout, WDT_ACTION action)
{
    WDT_Init(WDT_CLKSRC_IRC, (action == WDT_MRI)?WDT_MODE_INT_ONLY:WDT_MODE_RESET);
    WDT_Start(timeout);
    WDT_Feed();
}

void Watchdog::feed()
{
    WDT_Feed();
}

extern "C" void WDT_IRQHandler(void)
{
    WDT_ClrTimeOutFlag();
    WDT_Feed();
    __debugbreak();
}