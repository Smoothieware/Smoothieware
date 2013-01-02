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

#define ALPHA_AXIS 0
#define BETA_AXIS  1
#define GAMMA_AXIS 2

#define NOT_HOMING 0
#define MOVING_TO_ORIGIN_FAST 1
#define MOVING_BACK 2
#define MOVING_TO_ORIGIN_SLOW 3

#define alpha_min_endstop_checksum       28684
#define beta_min_endstop_checksum        23457
#define gamma_min_endstop_checksum       16137

#define alpha_max_endstop_checksum       31246
#define beta_max_endstop_checksum        26019
#define gamma_max_endstop_checksum       18699

#define alpha_fast_homing_rate_checksum  19720
#define beta_fast_homing_rate_checksum   9373
#define gamma_fast_homing_rate_checksum  3333

#define alpha_slow_homing_rate_checksum  45599
#define beta_slow_homing_rate_checksum   35252
#define gamma_slow_homing_rate_checksum  29212

#define alpha_homing_retract_checksum    4419
#define beta_homing_retract_checksum     48344
#define gamma_homing_retract_checksum    54848
#define endstop_debounce_count_checksum  25394


class Endstops : public Module{
    private:
        void wait_for_homed(char axes_to_move);

    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);

        StepperMotor* steppers[3];
        Pin*          pins[6];
        double  slow_rates[3];
        double  fast_rates[3];
        unsigned int  retract_steps[3];
        unsigned int  debounce_count;
        char status;
};

















#endif
