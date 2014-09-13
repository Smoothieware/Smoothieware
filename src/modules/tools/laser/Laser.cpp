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
#include "Config.h"
#include "StreamOutputPool.h"
#include "Block.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "libs/Pin.h"
#include "Gcode.h"
#include "PwmOut.h" // mbed.h lib

#define laser_module_enable_checksum        CHECKSUM("laser_module_enable")
#define laser_module_pin_checksum           CHECKSUM("laser_module_pin")
#define laser_module_pwm_period_checksum    CHECKSUM("laser_module_pwm_period")
#define laser_module_max_power_checksum     CHECKSUM("laser_module_max_power")
#define laser_module_tickle_power_checksum  CHECKSUM("laser_module_tickle_power")

Laser::Laser(){
}

void Laser::on_module_loaded() {
    if( !THEKERNEL->config->value( laser_module_enable_checksum )->by_default(false)->as_bool() ){
        // as not needed free up resource
        delete this;
        return;
    }

    // Get smoothie-style pin from config
    Pin* dummy_pin = new Pin();
    dummy_pin->from_string(THEKERNEL->config->value(laser_module_pin_checksum)->by_default("nc")->as_string())->as_output();

    laser_pin = NULL;

    // Get mBed-style pin from smoothie-style pin
    if( dummy_pin->port_number == 2 ){
        if( dummy_pin->pin == 0 ){ this->laser_pin = new mbed::PwmOut(p26); }
        if( dummy_pin->pin == 1 ){ this->laser_pin = new mbed::PwmOut(p25); }
        if( dummy_pin->pin == 2 ){ this->laser_pin = new mbed::PwmOut(p24); }
        if( dummy_pin->pin == 3 ){ this->laser_pin = new mbed::PwmOut(p23); }
        if( dummy_pin->pin == 4 ){ this->laser_pin = new mbed::PwmOut(p22); }
        if( dummy_pin->pin == 5 ){ this->laser_pin = new mbed::PwmOut(p21); }
    }

    if (laser_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Laser cannot use P%d.%d (P2.0 - P2.5 only). Laser module disabled.\n", dummy_pin->port_number, dummy_pin->pin);
        delete dummy_pin;
        delete this;
        return;
    }

    this->laser_inverting = dummy_pin->inverting;

    delete dummy_pin;
    dummy_pin = NULL;

    this->laser_pin->period_us(THEKERNEL->config->value(laser_module_pwm_period_checksum)->by_default(20)->as_number());
    this->laser_pin->write(this->laser_inverting ? 1 : 0);

    this->laser_max_power =    THEKERNEL->config->value(laser_module_max_power_checksum   )->by_default(0.8f)->as_number() ;
    this->laser_tickle_power = THEKERNEL->config->value(laser_module_tickle_power_checksum)->by_default(0   )->as_number() ;

    //register for events
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
}

// Turn laser off laser at the end of a move
void  Laser::on_block_end(void* argument){
    this->laser_pin->write(this->laser_inverting ? 1 : 0);
}

// Set laser power at the beginning of a block
void Laser::on_block_begin(void* argument){
    this->set_proportional_power();
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Laser::on_pause(void* argument){
    this->laser_pin->write(this->laser_inverting ? 1 : 0);
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Laser::on_play(void* argument){
    this->set_proportional_power();
}

// Turn laser on/off depending on received GCodes
void Laser::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    this->laser_on = false;
    if( gcode->has_g){
        int code = gcode->g;
        if( code == 0 ){                    // G0
            this->laser_pin->write(this->laser_inverting ? 1 - this->laser_tickle_power : this->laser_tickle_power);
            this->laser_on =  false;
        }else if( code >= 1 && code <= 3 ){ // G1, G2, G3
            this->laser_on =  true;
        }
    }
    if ( gcode->has_letter('S' )){
        this->laser_max_power = gcode->get_value('S');
//         THEKERNEL->streams->printf("Adjusted laser power to %d/100\r\n",(int)(this->laser_max_power*100.0+0.5));
    }

}

// We follow the stepper module here, so speed must be proportional
void Laser::on_speed_change(void* argument){
    if( this->laser_on ){
        this->set_proportional_power();
    }
}

void Laser::set_proportional_power(){
    if( this->laser_on && THEKERNEL->stepper->get_current_block() ){
        // adjust power to maximum power and actual velocity
        float proportional_power = float(float(this->laser_max_power) * float(THEKERNEL->stepper->get_trapezoid_adjusted_rate()) / float(THEKERNEL->stepper->get_current_block()->nominal_rate));
        this->laser_pin->write(this->laser_inverting ? 1 - proportional_power : proportional_power);
    }
}
