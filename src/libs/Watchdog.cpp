#include "Watchdog.h"
#include "Kernel.h"

#include <lpc17xx_wdt.h>

#include <mri.h>

#include "gpio.h"
extern GPIO leds[];

// TODO : comment this
// Basically, when stuff stop answering, reset, or enter MRI mode, or something

Watchdog::Watchdog(uint32_t timeout, WDT_ACTION action)
{
    WDT_Init(WDT_CLKSRC_IRC, (action == WDT_MRI)?WDT_MODE_INT_ONLY:WDT_MODE_RESET);
    WDT_Start(timeout);
    WDT_Feed();
    if(action == WDT_MRI) {
        // enable the interrupt
        NVIC_EnableIRQ(WDT_IRQn);
        NVIC_SetPriority(WDT_IRQn, 1);
    }
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


// when watchdog triggers, set a led pattern and enter MRI which turns everything off into a safe state
// TODO handle when MRI is disabled
extern "C" void WDT_IRQHandler(void)
{
    if(THEKERNEL->is_using_leds()) {
        // set led pattern to show we are in watchdog timeout
        leds[0]= 0;
        leds[1]= 1;
        leds[2]= 0;
        leds[3]= 1;
    }

    WDT_ClrTimeOutFlag(); // bootloader uses this flag to enter DFU mode
    WDT_Feed();
    __debugbreak();
}
