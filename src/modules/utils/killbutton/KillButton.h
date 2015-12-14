#pragma once

#include "libs/Pin.h"

class KillButton : public Module {
    public:
        KillButton();

        void on_module_loaded();
        void on_idle(void *argument);
        uint32_t button_tick(uint32_t dummy);

    private:
        Pin kill_button;
        enum STATE {
            IDLE,
            KILL_BUTTON_DOWN,
            KILLED_BUTTON_DOWN,
            KILLED_BUTTON_UP,
            UNKILL_BUTTON_DOWN,
            UNKILL_TIMING_BUTTON_DOWN,
            UNKILL_FIRE,
            UNKILLED_BUTTON_DOWN
        };

        struct {
            uint8_t unkill_timer:6;
            volatile STATE state:4;
            bool unkill_enable:1;
        };
};
