#include "Watchdog.h"

#include <lpc17xx_wdt.h>

#include <mri.h>

extern "C" {
    static uint32_t _set_high_on_debug[5] = {
        (1 << 4) | (1 << 10) | (1 << 19) | (1 << 21),
        0,
        0,
        0,
        0
    };
    static uint32_t _set_low_on_debug[5]  = {
        0,
        0,
        (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7),
        0,
        0
    };

    static LPC_GPIO_TypeDef* io;
    static int i;

    void __mriPlatform_EnteringDebuggerHook()
    {
        for (i = 0; i < 5; i++)
        {
            io           = (LPC_GPIO_TypeDef*) (LPC_GPIO_BASE + (0x20 * i));
            io->FIOMASK &= ~(_set_high_on_debug[i] | _set_low_on_debug[i]);
            io->FIOSET   = _set_high_on_debug[i];
            io->FIOCLR   = _set_low_on_debug[i];
        }
    }

    void __mriPlatform_LeavingDebuggerHook()
    {
    }
}

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
    WDT_ClrTimeOutFlag();
    WDT_Feed();
    __debugbreak();
}
