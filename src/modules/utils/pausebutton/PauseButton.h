#ifndef PAUSEBUTTON_H
#define PAUSEBUTTON_H

#include "libs/Pin.h"

class PauseButton : public Module {
    public:
        PauseButton();

        void on_module_loaded();
        void on_console_line_received( void *argument );
        void on_idle(void *argument);
        uint32_t button_tick(uint32_t dummy);

    private:
        Pin pause_button;
        Pin kill_button;
        struct {
            bool pause_enable:1;
            bool kill_enable:1;
            bool button_state:1;
            bool killed:1;
            volatile bool do_kill:1;
        };
};

#endif
