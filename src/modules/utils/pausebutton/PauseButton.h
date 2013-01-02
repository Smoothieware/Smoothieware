#ifndef PAUSEBUTTON_H
#define PAUSEBUTTON_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"

#define pause_button_enable_checksum 55526
#define pause_button_pin_checksum 32709
#define pause_led_pin_checksum    48477

class PauseButton : public Module {
    public:
        PauseButton();
       
        void on_module_loaded();
        uint32_t button_tick(uint32_t dummy);
        void on_play( void* argument );
        void on_pause( void* argument );
        
        bool       enable;
        Pin*       button;
        Pin*       led;
        bool       button_state;
        bool       play_state;
};











#endif
