#include "mbed.h"
#include "libs/Kernel.h"
#include "Pauser.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

Pauser::Pauser(PinName ButtonPin, PinName LedPin) : button(ButtonPin), led(LedPin) {}

void Pauser::on_module_loaded(){
    this->button_state = false;
    this->led = false;
    this->play_state   = true;
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->button_ticker.attach_us(this, &Pauser::button_tick, 1000000/100);
}

//TODO: Make this use InterruptIn
//Check the state of the button and ask accordingly
void Pauser::button_tick(){
    if(this->button_state != this->button){ 
        this->button_state = this->button; 
        if( this->button_state ){
            if( this->play_state ){
                this->play_state = false;
                this->kernel->call_event(ON_PAUSE, this);
            }else{
                this->play_state = true;
                this->kernel->call_event(ON_PLAY, this);
            } 
        } 
    }
}

void Pauser::on_play( void* argument ){
    this->play_state = true;
    this->led = false;
}

void Pauser::on_pause( void* argument ){
    this->play_state = false;
    this->led = true;
}

