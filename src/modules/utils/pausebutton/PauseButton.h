#ifndef PAUSEBUTTON_H
#define PAUSEBUTTON_H

#include "libs/Pin.h"

class PauseButton : public Module {
    public:
        PauseButton();

        void on_module_loaded();
        void on_console_line_received( void *argument );
        uint32_t button_tick(uint32_t dummy);

    private:
        Pin button;
        struct {
            bool enable:1;
            bool button_state:1;
        };
};

#endif
