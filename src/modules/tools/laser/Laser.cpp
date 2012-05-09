/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Stepper.h"
#include "Laser.h"
#include "libs/nuts_bolts.h"

Laser::Laser(PinName pin) : laser_pin(pin){
    this->laser_pin.period_us(10);
}

void Laser::on_module_loaded() {
    if( !this->kernel->config->value( laser_module_enable_checksum )->by_default(false)->as_bool() ){ return; } 
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
}

// Turn laser off laser at the end of a move
void  Laser::on_block_end(void* argument){
    this->laser_pin = 0;
}

// Set laser power at the beginning of a block
void Laser::on_block_begin(void* argument){
    this->set_proportional_power();
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Laser::on_pause(void* argument){
    this->laser_pin = 0;
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Laser::on_play(void* argument){
    this->set_proportional_power();
}

// Turn laser on/off depending on received GCodes
void Laser::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    this->laser_on = false;
    if( gcode->has_letter('G' )){
        int code = gcode->get_value('G');
        if( code == 0 ){                    // G0
            this->laser_pin = 0;
            this->laser_on =  false;
        }else if( code >= 1 && code <= 3 ){ // G1, G2, G3
            this->laser_on =  true;
        }
    }
}

// We follow the stepper module here, so speed must be proportional
void Laser::on_speed_change(void* argument){
    this->set_proportional_power();
}

void Laser::set_proportional_power(){
    if( this->laser_on && this->kernel->stepper->current_block ){ 
        this->laser_pin = double(this->kernel->stepper->trapezoid_adjusted_rate)/double(this->kernel->stepper->current_block->nominal_rate);
    }
}
