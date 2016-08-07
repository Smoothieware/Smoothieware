/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Laser.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "Block.h"
#include "SlowTicker.h"
#include "Robot.h"

#include "libs/Pin.h"
#include "Gcode.h"
#include "PwmOut.h" // mbed.h lib

#include <algorithm>

#define laser_module_enable_checksum            CHECKSUM("laser_module_enable")
#define laser_module_pin_checksum               CHECKSUM("laser_module_pin")
#define laser_module_pwm_pin_checksum           CHECKSUM("laser_module_pwm_pin")
#define laser_module_ttl_pin_checksum           CHECKSUM("laser_module_ttl_pin")
#define laser_module_pwm_period_checksum        CHECKSUM("laser_module_pwm_period")
#define laser_module_maximum_power_checksum     CHECKSUM("laser_module_maximum_power")
#define laser_module_minimum_power_checksum     CHECKSUM("laser_module_minimum_power")
#define laser_module_tickle_power_checksum      CHECKSUM("laser_module_tickle_power")
#define laser_module_max_power_checksum         CHECKSUM("laser_module_max_power")
#define laser_module_maximum_s_value_checksum   CHECKSUM("laser_module_maximum_s_value")


Laser::Laser()
{
    laser_on = false;
}

void Laser::on_module_loaded()
{
    if( !THEKERNEL->config->value( laser_module_enable_checksum )->by_default(false)->as_bool() ) {
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

    if (pwm_pin == NULL) {
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


    uint32_t period= THEKERNEL->config->value(laser_module_pwm_period_checksum)->by_default(20)->as_number();
    this->pwm_pin->period_us(period);
    this->pwm_pin->write(this->pwm_inverting ? 1 : 0);
    this->laser_maximum_power = THEKERNEL->config->value(laser_module_maximum_power_checksum)->by_default(1.0f)->as_number() ;

    // These config variables are deprecated, they have been replaced with laser_module_default_power and laser_module_minimum_power
    this->laser_minimum_power = THEKERNEL->config->value(laser_module_tickle_power_checksum)->by_default(0)->as_number() ;

    // Load in our preferred config variables
    this->laser_minimum_power = THEKERNEL->config->value(laser_module_minimum_power_checksum)->by_default(this->laser_minimum_power)->as_number() ;

    // S value that represents maximum (default 1)
    this->laser_maximum_s_value = THEKERNEL->config->value(laser_module_maximum_s_value_checksum)->by_default(1.0f)->as_number() ;

    turn_laser_off();

    //register for events
    this->register_for_event(ON_HALT);

    // no point in updating the power more than the PWM frequency, but no more than 1KHz
    THEKERNEL->slow_ticker->attach(std::min(1000UL, 1000000/period), this, &Laser::set_proportional_power);
}

void Laser::turn_laser_off()
{
    this->pwm_pin->write(this->pwm_inverting ? 1 : 0);
    if (this->ttl_used) this->ttl_pin->set(false);
    laser_on = false;
}

// calculates the current speed ratio from the currently executing block
float Laser::current_speed_ratio(const Block *block) const
{
    // find the primary moving actuator (the one with the most steps)
    size_t pm= 0;
    uint32_t max_steps= 0;
    for (size_t i = 0; i < THEROBOT->get_number_registered_motors(); i++) {
        // find the motor with the most steps
        if(block->steps[i] > max_steps) {
            max_steps= block->steps[i];
            pm= i;
        }
    }

    // figure out the ratio of its speed, from 0 to 1 based on where it is on the trapezoid
    // FIXME steps_per_tick can change at any time, potential race condition if it changes while being read here
    float ratio= (float)block->tick_info[pm].steps_per_tick / block->tick_info[pm].plateau_rate;

    return ratio;
}

// get laser power for the currently executing block, returns false if nothing running or a G0
bool Laser::get_laser_power(float& power) const
{
    const Block *block = StepTicker::getInstance()->get_current_block();

    // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
    // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
    if(block != nullptr && block->is_ready && block->is_g123) {
        float requested_power = ((float)block->s_value/(1<<11)) / this->laser_maximum_s_value; // s_value is 1.11 Fixed point

        // Ensure we can't exceed maximum power
        if (requested_power < 0) requested_power = 0;
        else if (requested_power > 1) requested_power = 1;

        float ratio = current_speed_ratio(block);
        power = requested_power * ratio;

        return true;
    }

    return false;
}

// called every millisecond from timer ISR
uint32_t Laser::set_proportional_power(uint32_t dummy)
{
    float power;
    if(get_laser_power(power)) {
        // adjust power to maximum power and actual velocity
        float proportional_power = ( (this->laser_maximum_power - this->laser_minimum_power) * power ) + this->laser_minimum_power;
        this->pwm_pin->write(this->pwm_inverting ? 1 - proportional_power : proportional_power);
        if(!laser_on && this->ttl_used) this->ttl_pin->set(true);
        laser_on = true;

    } else if(laser_on) {
        // turn laser off
        turn_laser_off();
    }
    return 0;
}

void Laser::on_halt(void *argument)
{
    if(argument == nullptr) {
        turn_laser_off();
    }
}
