#pragma once

#include "libs/Pin.h"

class KillButton : public Module {
    public:
        KillButton();

        void on_module_loaded();
        void on_console_line_received( void *argument );
        void on_idle(void *argument);
        uint32_t button_tick(uint32_t dummy);

    private:
        Pin kill_button;
        struct {
            bool kill_enable:1;
            bool button_state:1;
            bool killed:1;
            volatile bool do_kill:1;
        };
};
