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

    this->enable     =  THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool();
    this->button.from_string( THEKERNEL->config->value( pause_button_pin_checksum )->by_default("2.12")->as_string())->as_input();

    THEKERNEL->slow_ticker->attach( 100, this, &PauseButton::button_tick );
}

//TODO: Make this use InterruptIn
//Check the state of the button and act accordingly
uint32_t PauseButton::button_tick(uint32_t dummy){
    if(!this->enable) return 0;
    // If button changed
    bool newstate = this->button.get();
    if(this->button_state != newstate){
        this->button_state = newstate;
        // If button pressed
        if( this->button_state ){
            if( this->play_state ){
                this->play_state = false;
                THEKERNEL->pauser->take();
            }else{
                this->play_state = true;
                THEKERNEL->pauser->release();
            }
        }
    }
    return 0;
}
