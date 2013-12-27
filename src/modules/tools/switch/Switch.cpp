/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "Switch.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"

#include "MRI_Hooks.h"

Switch::Switch(){}

Switch::Switch(uint16_t name){
    this->name_checksum = name;
    //this->dummy_stream = &(StreamOutput::NullStream);
}

void Switch::on_module_loaded(){
    this->input_pin_state = true;
    this->switch_state = true;
    this->switch_changed = false;

    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_MAIN_LOOP);

    // Settings
    this->on_config_reload(this);

    // input pin polling
    THEKERNEL->slow_ticker->attach( 100, this, &Switch::pinpoll_tick);

    // PWM
    THEKERNEL->slow_ticker->attach(1000, &output_pin, &Pwm::on_tick);
}


// Get config
void Switch::on_config_reload(void* argument){
    this->input_pin.from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_checksum    )->by_default("nc")->as_string())->as_input();
    this->input_pin_behavior     = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_behavior_checksum     )->by_default(momentary_checksum)->as_number();
    this->input_on_command     = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_on_command_checksum     )->by_default("")->as_string();
    this->input_off_command    = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_off_command_checksum    )->by_default("")->as_string();
    this->output_pin.from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum    )->by_default("nc")->as_string())->as_output();
    this->output_on_command     = THEKERNEL->config->value(switch_checksum, this->name_checksum, output_on_command_checksum     )->by_default("")->as_string();
    this->output_off_command    = THEKERNEL->config->value(switch_checksum, this->name_checksum, output_off_command_checksum    )->by_default("")->as_string();
    this->switch_state         = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_state_checksum )->by_default(false)->as_bool();
    this->switch_value         = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_value_checksum )->by_default(this->output_pin.max_pwm())->as_number();
    if(this->switch_state)
        this->output_pin.set(this->switch_value);
    else
        this->output_pin.set(0);

    set_low_on_debug(output_pin.port_number, output_pin.pin);
}


void Switch::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    // Add the gcode to the queue ourselves if we need it
    if( ( input_on_command.length() > 0 && ! gcode->command.compare(0, input_on_command.length(), input_on_command) )
        || ( input_off_command.length() > 0 && ! gcode->command.compare(0, input_off_command.length(), input_off_command) ) ){
        gcode->mark_as_taken();
        if( THEKERNEL->conveyor->queue.size() == 0 ){
            THEKERNEL->call_event(ON_GCODE_EXECUTE, gcode );
        }else{
            Block* block = THEKERNEL->conveyor->queue.get_ref( THEKERNEL->conveyor->queue.size() - 1 );
            block->append_gcode(gcode);
        }
    }
}

// Turn pin on and off
void Switch::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if(! gcode->command.compare(0, input_on_command.length(), input_on_command)){
        if (gcode->has_letter('S'))
        {
            int v = gcode->get_value('S') * output_pin.max_pwm() / 256.0;
            if (v) {
                this->output_pin.pwm(v);
                this->switch_value = v;
                this->switch_state = true;
            }
            else {
                this->output_pin.set(0);
                this->switch_state = false;
            }
        }
        else
        {
            // Turn pin on
            this->output_pin.pwm(this->switch_value);
            this->switch_state = true;
        }
    }
    else if(! gcode->command.compare(0, input_off_command.length(), input_off_command)){
        // Turn pin off
        this->output_pin.set(0);
        this->switch_state = false;
    }
}

void Switch::on_main_loop(void* argument){
    if(this->switch_changed){  
        if(this->switch_state){
            this->send_gcode( this->output_on_command, &(StreamOutput::NullStream) );
            this->output_pin.pwm(this->switch_value);
        }else{
            this->send_gcode( this->output_off_command, &(StreamOutput::NullStream) );
            this->output_pin.set(0);
        }
        this->switch_changed=false;      
    }
}

//TODO: Make this use InterruptIn
//Check the state of the button and act accordingly
uint32_t Switch::pinpoll_tick(uint32_t dummy){
    // If pin changed
    bool current_state = this->input_pin.get();
    if(this->input_pin_state != current_state){
        this->input_pin_state = current_state;
        // If pin high
        if( this->input_pin_state ){
            // if switch is a toggle switch
            if( this->input_pin_behavior == toggle_checksum ){
                this->flip();
            // else default is momentary
            }
            else{
                this->flip();
            }
        // else if button released
        }else{
            // if switch is momentary
            if( !this->input_pin_behavior == toggle_checksum ){
                this->flip();
            }
        }
    }
    return 0;
}

void Switch::flip(){
    this->switch_state = !this->switch_state;
    this->switch_changed = true;
}

void Switch::send_gcode(std::string msg, StreamOutput* stream) {
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}

