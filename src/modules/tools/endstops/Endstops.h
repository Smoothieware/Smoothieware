/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ENDSTOPS_MODULE_H
#define ENDSTOPS_MODULE_H

#include "libs/Module.h"
#include "libs/Pin.h"

#include <bitset>

class StepperMotor;

class Endstops : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);
        uint32_t acceleration_tick(uint32_t dummy);

    private:
        void do_homing(char axes_to_move);
        void do_homing_corexy(char axes_to_move);
        void wait_for_homed(char axes_to_move);
        void wait_for_homed_corexy(int axis);
        void corexy_home(int home_axis, bool dirx, bool diry, float fast_rate, float slow_rate, unsigned int retract_steps);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);

        float homing_position[3];
        float home_offset[3];
        std::bitset<3> home_direction;
        unsigned int  debounce_count;
        float  retract_mm[3];
        float  trim_mm[3];
        float  fast_rates[3];
        float  slow_rates[3];
        float  feed_rate[3];
        Pin    pins[6];
        char status;
        struct {
            bool is_corexy:1;
            bool is_delta:1;
        };
};

#endif
