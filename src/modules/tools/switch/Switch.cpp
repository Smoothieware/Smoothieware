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

Switch::Switch(){}

Switch::Switch(uint16_t name){
    this->name_checksum = name;
}

void Switch::on_module_loaded(){
    this->register_for_event(ON_GCODE_EXECUTE);
    
    // Settings
    this->on_config_reload(this);

}


// Get config
void Switch::on_config_reload(void* argument){
    this->on_m_code                   = this->kernel->config->value(switch_checksum, this->name_checksum, on_m_code_checksum          )->required()->as_number();
    this->off_m_code                  = this->kernel->config->value(switch_checksum, this->name_checksum, off_m_code_checksum         )->required()->as_number();
    this->output_pin                  = this->kernel->config->value(switch_checksum, this->name_checksum, output_pin_checksum         )->required()->as_pin()->as_output();
    this->output_pin->set(              this->kernel->config->value(switch_checksum, this->name_checksum, startup_state_checksum      )->by_default(0)->as_number() );
}

// Turn pin on and off
void Switch::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_letter('M' )){
        int code = gcode->get_value('M');
        if( code == this->on_m_code ){  
            // Turn pin on
            this->output_pin->set(1);
        } 
        if( code == this->off_m_code ){ 
            // Turn pin off 
            this->output_pin->set(0);
        } 
    }
}




