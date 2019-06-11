/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"

#include <stdint.h>

namespace mbed {
    class PwmOut;
}
class Pin;
class Block;

class Laser : public Module{
    public:
        Laser();
        virtual ~Laser() {};
        void on_module_loaded();
        void on_halt(void* argument);
        void on_suspend(void* argument);
        void on_gcode_received(void *argument);
        void on_console_line_received(void *argument);
        void on_get_public_data(void* argument);

        void set_scale(float s) { scale= s/100; }
        float get_scale() const { return scale*100; }
        bool set_laser_power(float p);
        float get_current_power() const;

    private:
        uint32_t set_proportional_power(uint32_t dummy);
        bool get_laser_power(float& power) const;
        float current_speed_ratio(const Block *block) const;

        mbed::PwmOut *pwm_pin;    // PWM output to regulate the laser power
        Pin *ttl_pin;				// TTL output to fire laser
        float laser_maximum_power; // maximum allowed laser power to be output on the pwm pin
        float laser_minimum_power; // value used to tickle the laser on moves.  Also minimum value for auto-scaling
        float laser_maximum_s_value; // Value of S code that will represent max power
        float scale;
        int32_t fire_duration; // manual fire command duration
        int32_t ms_per_tick; // ms between each ticks, depends on PWM frequency

        struct {
            bool laser_on:1;      // set if the laser is on
            bool pwm_inverting:1; // stores whether the PWM period should be inverted
            bool ttl_used:1;        // stores whether we have a TTL output
            bool ttl_inverting:1;   // stores whether the TTL output should be inverted
            bool manual_fire:1;     // set when manually firing
        };
};
