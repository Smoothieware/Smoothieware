#ifndef RESETWATCHDOG_H
#define RESETWATCHDOG_H

#include "lpc17xx_wdt.h"
#include "Module.h"

class ResetWatchdog : public Module {
    public:
        ResetWatchdog();

        void on_module_loaded();
        void on_gcode_received( void *argument );
        uint32_t watchdog_tick(uint32_t dummy);

    private:
        struct {
              bool awaked_from_reset:1;
              bool connected:1;
        };
};

#endif
