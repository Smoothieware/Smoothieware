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

#define laser_module_enable_checksum          	CHECKSUM("laser_module_enable")
#define laser_module_pin_checksum          	    CHECKSUM("laser_module_pin")
#define laser_module_pwm_pin_checksum          	CHECKSUM("laser_module_pwm_pin")
#define laser_module_ttl_pin_checksum    	   	CHECKSUM("laser_module_ttl_pin")
#define laser_module_pwm_period_checksum   	    CHECKSUM("laser_module_pwm_period")
#define laser_module_maximum_power_checksum    	CHECKSUM("laser_module_maximum_power")
#define laser_module_minimum_power_checksum     CHECKSUM("laser_module_minimum_power")
#define laser_module_default_power_checksum     CHECKSUM("laser_module_default_power")
#define laser_module_tickle_power_checksum      CHECKSUM("laser_module_tickle_power")
#define laser_module_max_power_checksum         CHECKSUM("laser_module_max_power")
#define laser_module_maximum_s_value_checksum   CHECKSUM("laser_module_maximum_s_value")


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

    // Alternative less ambiguous name for pwm_pin
    if (!dummy_pin->connected())
        dummy_pin->from_string(THEKERNEL->config->value(laser_module_pwm_pin_checksum)->by_default("nc")->as_string())->as_output();

    pwm_pin = dummy_pin->hardware_pwm();

    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Laser cannot use P%d.%d (P2.0 - P2.5, P1.18, P1.20, P1.21, P1.23, P1.24, P1.26, P3.25, P3.26 only). Laser module disabled.\n", dummy_pin->port_number, dummy_pin->pin);
        delete dummy_pin;
        delete this;
        return;
    }


    this->pwm_inverting = dummy_pin->is_inverting();

    delete dummy_pin;
    dummy_pin = NULL;

    // TTL settings
    this->ttl_pin = new Pin();
    ttl_pin->from_string( THEKERNEL->config->value(laser_module_ttl_pin_checksum)->by_default("nc" )->as_string())->as_output();
    this->ttl_used = ttl_pin->connected();
    this->ttl_inverting = ttl_pin->is_inverting();
    if (ttl_used) {
    	ttl_pin->set(0);
    } else {
    	delete ttl_pin;
    	ttl_pin = NULL;
    }


    this->pwm_pin->period_us(THEKERNEL->config->value(laser_module_pwm_period_checksum)->by_default(20)->as_number());
    this->pwm_pin->write(this->pwm_inverting ? 1 : 0);
    this->laser_maximum_power = THEKERNEL->config->value(laser_module_maximum_power_checksum)->by_default(1.0f)->as_number() ;

    // These config variables are deprecated, they have been replaced with laser_module_default_power and laser_module_minimum_power
    this->laser_minimum_power = THEKERNEL->config->value(laser_module_tickle_power_checksum)->by_default(0)->as_number() ;
    this->laser_power =         THEKERNEL->config->value(laser_module_max_power_checksum)->by_default(0.8f)->as_number() ;

    // Load in our preferred config variables
    this->laser_minimum_power = THEKERNEL->config->value(laser_module_minimum_power_checksum)->by_default(this->laser_minimum_power)->as_number() ;
    this->laser_power = THEKERNEL->config->value(laser_module_default_power_checksum)->by_default(this->laser_power)->as_number() ;

    // S value that represents maximum (default 1)
    this->laser_maximum_s_value = THEKERNEL->config->value(laser_module_maximum_s_value_checksum)->by_default(1.0f)->as_number() ;

    //register for events
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_HALT);
}

// Turn laser off laser at the end of a move
void  Laser::on_block_end(void* argument){
    this->pwm_pin->write(this->pwm_inverting ? 1 : 0);

    if (this->ttl_used) {
    	Block* block = static_cast<Block*>(argument);
    	// Only switch TTL off if this is the last block for this move - G2/3 are multiple blocks
    	if (block->final_rate == 0)
    		this->ttl_pin->set(0);
    }
}

// Set laser power at the beginning of a block
void Laser::on_block_begin(void* argument){
    this->set_proportional_power();

}

// Turn laser on/off depending on received GCodes
void Laser::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    this->laser_on = false;
    if( gcode->has_g){
        int code = gcode->g;
        if( code == 0 ){                    // G0
            this->pwm_pin->write(this->pwm_inverting ? 1 - this->laser_minimum_power : this->laser_minimum_power);
            this->laser_on =  false;
        }else if( code >= 1 && code <= 3 ){ // G1, G2, G3
            this->laser_on =  true;
        }
    }

    if ( gcode->has_letter('S' )){
    	float requested_power = gcode->get_value('S') / this->laser_maximum_s_value;
    	// Ensure we can't exceed maximum power
    	if (requested_power > 1)
    		requested_power = 1;

        this->laser_power = requested_power;
    }

    if (this->ttl_used)
        this->ttl_pin->set(this->laser_on);

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
        float proportional_power = (((this->laser_maximum_power-this->laser_minimum_power)*(this->laser_power * THEKERNEL->stepper->get_trapezoid_adjusted_rate() / THEKERNEL->stepper->get_current_block()->nominal_rate))+this->laser_minimum_power);
        this->pwm_pin->write(this->pwm_inverting ? 1 - proportional_power : proportional_power);
    }
}

void Laser::on_halt(void *argument)
{
    if(argument == nullptr) {
    	// Safety check - turn laser off on halt
    	this->laser_on = false;
    	if (this->ttl_used)
    	        this->ttl_pin->set(this->laser_on);
    }
}
