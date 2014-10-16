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
    class AnalogOut;
}

class Laser : public Module{
    public:
        Laser();
        virtual ~Laser() {};
        void on_module_loaded();
        void on_block_end(void* argument);
        void on_block_begin(void* argument);
        void on_play(void* argument);
        void on_pause(void* argument);
        void on_halt(void* argument);
        void on_gcode_execute(void* argument);
        void on_speed_change(void* argument);

    private:
        void set_proportional_power();

        union laser_pin_t {
            mbed::PwmOut*    pwm;    // PWM output to regulate the laser power
            mbed::AnalogOut* dac;      // DAC output to regulate the laser power
        } laser_pin;

        struct {
            bool laser_on:1;     // Laser status
            bool laser_inverting:1; // stores whether the pwm period should be inverted
            bool laser_control_type_pwm:1; // true for pwm, false for dac output
        };
        float            laser_max_power; // initial setpoint used if one is not specified in G1 Snnn
        float            laser_tickle_power; // value used to tickle the laser on moves
};

#endif
