/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Stepper.h"
#include "Laser.h"

Laser::Laser(PinName pin) : laser_pin(pin){
    this->laser_pin.period_us(10);
}

void Laser::on_module_loaded() {
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
}

// Turn laser on/off depending on received GCodes
void Laser::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_letter('G' )){
        int code = gcode->get_value('G');
        if( code == 0 ){                  // G0
            this->laser_pin = 0;
            this->laser_on =  false;
        }else if( code > 0 && code < 4 ){ // G1, G2, G3
            this->laser_on =  true;
        }
    }
}

void Laser::on_speed_change(void* argument){
    Stepper* stepper = static_cast<Stepper*>(argument);
    if( this->laser_on ){ 
        this->laser_pin = double(stepper->trapezoid_adjusted_rate)/double(stepper->current_block->nominal_rate);
    }
}
