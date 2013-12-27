/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ENDSTOPS_MODULE_H
#define ENDSTOPS_MODULE_H

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "libs/StepperMotor.h"
#include "libs/Pin.h"


class Endstops : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);

    private:
        void do_homing(char axes_to_move);
        void do_homing_corexy(char axes_to_move);
        void wait_for_homed(char axes_to_move);
        void wait_for_homed_corexy(int axis);
        void corexy_home(int home_axis, bool dirx, bool diry, float fast_rate, float slow_rate, unsigned int retract_steps);
        void trim2mm(float * mm);

        float steps_per_mm[3];
        float homing_position[3];
        float home_offset[3];
        bool home_direction[3];
        unsigned int  debounce_count;
        unsigned int  retract_steps[3];
        int  trim[3];
        float  fast_rates[3];
        float  slow_rates[3];
        Pin           pins[6];
        StepperMotor* steppers[3];
        char status;
        bool is_corexy;
        bool is_delta;
};

#endif
