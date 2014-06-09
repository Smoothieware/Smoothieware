#ifndef PAUSEBUTTON_H
#define PAUSEBUTTON_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"

#define pause_button_enable_checksum CHECKSUM("pause_button_enable")
#define pause_button_pin_checksum    CHECKSUM("pause_button_pin")
#define freeze_command_checksum      CHECKSUM("freeze")
#define unfreeze_command_checksum    CHECKSUM("unfreeze")

class PauseButton;
class PauseButton : public Module {
    public:
        PauseButton();

        void on_module_loaded();
        void on_console_line_received( void *argument );
        uint32_t button_tick(uint32_t dummy);

    private:
        Pin        button;
        bool       enable;
        bool       button_state;
        bool       play_state;
};

#endif
