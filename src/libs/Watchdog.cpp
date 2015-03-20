#include "Watchdog.h"

#include <mri.h>

// TODO : comment this
// Basically, when stuff stop answering, reset, or enter MRI mode, or something

Watchdog::Watchdog(uint32_t timeout, WDT_ACTION action)
{
        /* FIXME STM32 
    WDT_Init(WDT_CLKSRC_IRC, (action == WDT_MRI)?WDT_MODE_INT_ONLY:WDT_MODE_RESET);
    WDT_Start(timeout);
    WDT_Feed();
    * */
}

void Watchdog::feed()
{
        /* FIXME STM32 
    WDT_Feed();
    * */
}

void Watchdog::on_module_loaded()
{
    register_for_event(ON_IDLE);
    feed();
}

void Watchdog::on_idle(void*)
{
    feed();
}


extern "C" void WDT_IRQHandler(void)
{
        /* FIXME STM32 
    WDT_ClrTimeOutFlag();
    WDT_Feed();
    */
    __debugbreak();
}
