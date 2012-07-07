#include "libs/Kernel.h"
#include "PauseButton.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;

PauseButton::PauseButton(){}

void PauseButton::on_module_loaded(){
    this->button_state = true;
    this->play_state   = true;
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);

    this->button     =  this->kernel->config->value( pause_button_pin_checksum )->by_default("2.12")->as_pin()->as_input();
    this->led        =  this->kernel->config->value( pause_led_pin_checksum    )->by_default("4.28")->as_pin()->as_output();

    this->kernel->slow_ticker->attach( 100, this, &PauseButton::button_tick );
}

//TODO: Make this use InterruptIn
//Check the state of the button and act accordingly
uint32_t PauseButton::button_tick(uint32_t dummy){
    // If button changed 
    if(this->button_state != this->button->get()){ 
        this->button_state = this->button->get();
        // If button pressed 
        if( this->button_state ){
            if( this->play_state ){
                this->play_state = false;
                this->kernel->pauser->take(); 
            }else{
                this->play_state = true;
                this->kernel->pauser->release(); 
            } 
        } 
    }
}

void PauseButton::on_play( void* argument ){
    this->led->set(0);
}

void PauseButton::on_pause( void* argument ){
    this->led->set(1);
}

