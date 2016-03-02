/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LASER_MODULE_H
#define LASER_MODULE_H

#include "libs/Module.h"

namespace mbed {
    class PwmOut;
}
class Pin;

class Laser : public Module{
    public:
        Laser();
        virtual ~Laser() {};
        void on_module_loaded();
        void on_block_end(void* argument);
        void on_block_begin(void* argument);
        void on_gcode_execute(void* argument);
        void on_speed_change(void* argument);
        void on_halt(void* argument);

    private:
        void set_proportional_power();
        mbed::PwmOut *pwm_pin;    // PWM output to regulate the laser power
        Pin *ttl_pin;				// TTL output to fire laser
        struct {
            bool laser_on:1;        // Laser status
            bool pwm_inverting:1; // stores whether the PWM period should be inverted
            bool ttl_used:1;		// stores whether we have a TTL output
            bool ttl_inverting:1;   // stores whether the TTL output should be inverted
        };
        float            laser_maximum_power; // maximum allowed laser power to be output on the pwm pin
        float            laser_minimum_power; // value used to tickle the laser on moves.  Also minimum value for auto-scaling
        float            laser_power;     // current laser power
        float            laser_maximum_s_value; // Value of S code that will represent max power
};

#endif
