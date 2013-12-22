#include "MRI_Hooks.h"

#include <sLPC17xx.h>
#include <mri.h>

// This is used by MRI to turn pins on and off when entering and leaving MRI. Useful for not burning everything down
// See http://smoothieware.org/mri-debugging 

extern "C" {
    static uint32_t _set_high_on_debug[5] = {
//         (1 << 4) | (1 << 10) | (1 << 19) | (1 << 21), // smoothieboard stepper EN pins
        0,
        0,
        0,
        0,
        0
    };
    static uint32_t _set_low_on_debug[5]  = {
        0,
        0,
//         (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7), // smoothieboard heater outputs
        0,
        0,
        0
    };

    static uint32_t _previous_state[5];

    static LPC_GPIO_TypeDef* io;
    static int i;

    void __mriPlatform_EnteringDebuggerHook()
    {
        for (i = 0; i < 5; i++)
        {
            io           = (LPC_GPIO_TypeDef*) (LPC_GPIO_BASE + (0x20 * i));
            io->FIOMASK &= ~(_set_high_on_debug[i] | _set_low_on_debug[i]);

            _previous_state[i] = io->FIOPIN;

            io->FIOSET   = _set_high_on_debug[i];
            io->FIOCLR   = _set_low_on_debug[i];
        }
    }

    void __mriPlatform_LeavingDebuggerHook()
    {
        for (i = 0; i < 5; i++)
        {
            io           = (LPC_GPIO_TypeDef*) (LPC_GPIO_BASE + (0x20 * i));
            io->FIOMASK &= ~(_set_high_on_debug[i] | _set_low_on_debug[i]);
            io->FIOSET   =   _previous_state[i]  & (_set_high_on_debug[i] | _set_low_on_debug[i]);
            io->FIOCLR   = (~_previous_state[i]) & (_set_high_on_debug[i] | _set_low_on_debug[i]);
        }
    }

    void set_high_on_debug(int port, int pin)
    {
        if ((port >= 5) || (port < 0))
            return;
        if ((pin >= 32) || (pin < 0))
            return;
        _set_high_on_debug[port] |= (1<<pin);
    }

    void set_low_on_debug(int port, int pin)
    {
        if ((port >= 5) || (port < 0))
            return;
        if ((pin >= 32) || (pin < 0))
            return;
        _set_low_on_debug[port] |= (1<<pin);
    }
}
