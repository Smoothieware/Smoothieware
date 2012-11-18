#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "Button.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;

//Button::Button(){}

Button::Button(uint16_t name){
    this->name_checksum = name;
    this->dummy_stream = new StreamOutput();
}

void Button::on_module_loaded(){
    this->button_state = true;
    this->switch_state = true;

    // Settings
    this->on_config_reload(this);

    this->kernel->slow_ticker->attach( 100, this, &Button::button_tick );
}

void Button::on_config_reload(void* argument){
    this->toggle                      = this->kernel->config->value( button_checksum, this->name_checksum, toggle_checksum )->by_default(false)->as_bool();
    this->button                      = this->kernel->config->value( button_checksum, this->name_checksum, input_pin_checksum )->by_default("2.12")->as_pin()->as_input();
    this->on_m_code                   = this->kernel->config->value( button_checksum, this->name_checksum, on_m_code_checksum          )->required()->as_number();
    this->off_m_code                  = this->kernel->config->value( button_checksum, this->name_checksum, off_m_code_checksum         )->required()->as_number();
}

//TODO: Make this use InterruptIn
//Check the state of the button and act accordingly
uint32_t Button::button_tick(uint32_t dummy){
//    if(!this->enable) return 0;
    // If button changed
    if(this->button_state != this->button->get()){
        this->button_state = this->button->get();
        // If button pressed
        if( this->button_state ){
            if( this->toggle ){
                this->switch_state = !this->switch_state;
                if( this->switch_state ){
//                    this->send_gcode( "M" + itoa( this->on_m_code ), dummy_stream );
                }else{
 //                   this->send_gcode( "M" + itoa( this->off_m_code ), dummy_stream );
                }
            }else{
//                this->send_gcode( "M" + itoa( this->on_m_code ), dummy_stream );
            }
        }else{
            if( !this->toggle ){
//                this->send_gcode( "M" + itoa( this->off_m_code ), dummy_stream );
            }
        }
    }
    return 0;
}

void Button::send_gcode(std::string msg, StreamOutput* stream) {
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}
