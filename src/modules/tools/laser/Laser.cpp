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
    
    this->coolant_good = 1;
    this->coolant_paused = this->coolant_msg_state = false;
    this->laser_coolant_pin.from_string( THEKERNEL->config->value(laser_coolant_sensor_pin)->by_default("nc")->as_string() )->as_input()->rising_interrupt();
    this->laser_coolant_freq = THEKERNEL->config->value(laser_coolant_sensor_frequency)->by_default(1)->as_int();
    if ( this->laser_coolant_pin.connected() && this->laser_coolant_freq > 0 ){
        if ( !this->laser_coolant_pin.supports_interrupt() ){
            // Note: this check should be after the laser pin pwm is initialized, so that we don't leave the laser stuck on!
            THEKERNEL->streams->printf("Error: Laser coolant flow sensor must be on port 0 or 2, not P%d. "
                "Laser module disabled.\n", this->laser_coolant_pin.port_number);
            delete this;
            return;
        }
        
        THEKERNEL->slow_ticker->attach( this->laser_coolant_freq, this, &Laser::coolant_check );
    }

    //register for events
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_SPEED_CHANGE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_MAIN_LOOP);
}

// Send coolant-related messages here, since they are actually generated in an interrupt context that can't print
void Laser::on_main_loop(void* argument){
    if ( this->coolant_paused != this->coolant_msg_state ) {
        if ( this->coolant_paused ){
            THEKERNEL->streams->printf("Error: coolant flow failure! Will restart after 15 seconds of good flow.\n");
        } else {
            THEKERNEL->streams->printf("Coolant flow has resumed, continuing.\n");
        }
        this->coolant_msg_state = this->coolant_paused;
    }
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
    if( this->laser_on && THEKERNEL->stepper->current_block ){
        // adjust power to maximum power and actual velocity
        float proportional_power = float(float(this->laser_max_power) * float(THEKERNEL->stepper->trapezoid_adjusted_rate) / float(THEKERNEL->stepper->current_block->nominal_rate));
        this->laser_pin->write(this->laser_inverting ? 1 - proportional_power : proportional_power);
    }
}

uint32_t Laser::coolant_check(uint32_t dummy){
    if ( this->laser_coolant_pin.rising_edge_seen() ){
        this->laser_coolant_pin.clear_interrupt();
        this->coolant_good += (this->coolant_good < 0xFFFFFFFF);     // increment without wraparound
        if ( this->coolant_paused && (this->coolant_good >= 15 * this->laser_coolant_freq) ){
            THEKERNEL->pauser->release();
            this->coolant_paused = false;
        }
    } else {
        this->coolant_good = 0;
        if ( !this->coolant_paused ){
            THEKERNEL->pauser->take();
            this->coolant_paused = true;
            this->laser_pin->write(this->laser_inverting ? 1 : 0);
        }
    }
    return 0;
}
