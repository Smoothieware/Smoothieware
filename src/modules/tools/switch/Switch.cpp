/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "Switch.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"

#include "MRI_Hooks.h"

Switch::Switch(){}

Switch::Switch(uint16_t name){
    this->name_checksum = name;
}

void Switch::on_module_loaded(){
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);

    // Settings
    this->on_config_reload(this);

    // PWM
    this->kernel->slow_ticker->attach(1000, &output_pin, &Pwm::on_tick);
}


// Get config
void Switch::on_config_reload(void* argument){
    this->on_m_code     = this->kernel->config->value(switch_checksum, this->name_checksum, on_m_code_checksum     )->required()->as_number();
    this->off_m_code    = this->kernel->config->value(switch_checksum, this->name_checksum, off_m_code_checksum    )->required()->as_number();
    this->output_pin.from_string(this->kernel->config->value(switch_checksum, this->name_checksum, output_pin_checksum    )->required()->as_string())->as_output();
    this->output_pin.set(this->kernel->config->value(switch_checksum, this->name_checksum, startup_state_checksum )->by_default(0)->as_number() );

    set_low_on_debug(output_pin.port_number, output_pin.pin);
}


void Switch::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    // Add the gcode to the queue ourselves if we need it
    if( gcode->has_m && ( gcode->m == this->on_m_code || gcode->m == this->off_m_code ) ){
        gcode->mark_as_taken();
        if( this->kernel->conveyor->queue.size() == 0 ){
            this->kernel->call_event(ON_GCODE_EXECUTE, gcode );
        }else{
            Block* block = this->kernel->conveyor->queue.get_ref( this->kernel->conveyor->queue.size() - 1 );
            block->append_gcode(gcode);
        }
    }
}

// Turn pin on and off
void Switch::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_m){
        int code = gcode->m;
        if( code == this->on_m_code ){
            if (gcode->has_letter('S'))
            {
                int v = gcode->get_value('S') * output_pin.max_pwm() / 256.0;
                if (v)
                    this->output_pin.pwm(v);
                else
                    this->output_pin.set(0);
            }
            else
            {
                // Turn pin on
                this->output_pin.set(1);
            }
        }
        if( code == this->off_m_code ){
            // Turn pin off
            this->output_pin.set(0);
        }
    }
}
