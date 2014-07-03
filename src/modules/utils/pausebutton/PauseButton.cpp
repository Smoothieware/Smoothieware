#include "libs/Kernel.h"
#include "PauseButton.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "SlowTicker.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "Pauser.h"
#include "checksumm.h"
#include "ConfigValue.h"

using namespace std;

#define pause_button_enable_checksum CHECKSUM("pause_button_enable")
#define pause_button_pin_checksum    CHECKSUM("pause_button_pin")
#define freeze_command_checksum      CHECKSUM("freeze")
#define unfreeze_command_checksum    CHECKSUM("unfreeze")

PauseButton::PauseButton(){}

void PauseButton::on_module_loaded(){
    this->button_state = true;
    this->play_state   = true;

    this->enable     =  THEKERNEL->config->value( pause_button_enable_checksum )->by_default(false)->as_bool();
    this->button.from_string( THEKERNEL->config->value( pause_button_pin_checksum )->by_default("2.12")->as_string())->as_input();

    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);

    if(this->enable) THEKERNEL->slow_ticker->attach( 100, this, &PauseButton::button_tick );
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

// When a new line is received, check if it is a command, and if it is, act upon it
void PauseButton::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    // ignore comments and blank lines and if this is a G code then also ignore it
    char first_char = new_message.message[0];
    if(strchr(";( \n\rGMTN", first_char) != NULL) return;

    int checksum = get_checksum(shift_parameter(new_message.message));

    if (checksum == freeze_command_checksum) {
        if( this->play_state ){
            this->play_state = false;
            THEKERNEL->pauser->take();
        }
    }
    else if (checksum == unfreeze_command_checksum) {
        if( ! this->play_state ){
            this->play_state = true;
            THEKERNEL->pauser->release();
        }
    }
}

