#ifndef pauser_h
#define pauser_h

#include "mbed.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"


class Pauser : public Module {
    public:
        Pauser(PinName ButtonPin, PinName LedPin);
       
        void on_module_loaded();
        void button_tick();
        void on_play( void* argument );
        void on_pause( void* argument );
        
        DigitalIn  button;
        DigitalOut led; 
        Ticker     button_ticker;
        bool       button_state;
        bool       play_state;
};











#endif
