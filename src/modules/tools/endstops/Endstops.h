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

#define endstops_module_enable_checksum         CHECKSUM("endstops_enable")

#define alpha_min_endstop_checksum       CHECKSUM("alpha_min_endstop")
#define beta_min_endstop_checksum        CHECKSUM("beta_min_endstop")
#define gamma_min_endstop_checksum       CHECKSUM("gamma_min_endstop")

#define alpha_max_endstop_checksum       CHECKSUM("alpha_max_endstop")
#define beta_max_endstop_checksum        CHECKSUM("beta_max_endstop")
#define gamma_max_endstop_checksum       CHECKSUM("gamma_max_endstop")

#define alpha_trim_checksum       CHECKSUM("alpha_trim")
#define beta_trim_checksum        CHECKSUM("beta_trim")
#define gamma_trim_checksum       CHECKSUM("gamma_trim")

#define alpha_fast_homing_rate_checksum  CHECKSUM("alpha_fast_homing_rate")
#define beta_fast_homing_rate_checksum   CHECKSUM("beta_fast_homing_rate")
#define gamma_fast_homing_rate_checksum  CHECKSUM("gamma_fast_homing_rate")

#define alpha_slow_homing_rate_checksum  CHECKSUM("alpha_slow_homing_rate")
#define beta_slow_homing_rate_checksum   CHECKSUM("beta_slow_homing_rate")
#define gamma_slow_homing_rate_checksum  CHECKSUM("gamma_slow_homing_rate")

#define alpha_homing_retract_checksum    CHECKSUM("alpha_homing_retract")
#define beta_homing_retract_checksum     CHECKSUM("beta_homing_retract")
#define gamma_homing_retract_checksum    CHECKSUM("gamma_homing_retract")
#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")

#define alpha_homing_direction_checksum  CHECKSUM("alpha_homing_direction")
#define beta_homing_direction_checksum   CHECKSUM("beta_homing_direction")
#define gamma_homing_direction_checksum  CHECKSUM("gamma_homing_direction")
#define home_to_max_checksum             CHECKSUM("home_to_max")
#define home_to_min_checksum             CHECKSUM("home_to_min")
#define alpha_min_checksum               CHECKSUM("alpha_min")
#define beta_min_checksum                CHECKSUM("beta_min")
#define gamma_min_checksum               CHECKSUM("gamma_min")

#define alpha_max_checksum               CHECKSUM("alpha_max")
#define beta_max_checksum                CHECKSUM("beta_max")
#define gamma_max_checksum               CHECKSUM("gamma_max")


class Endstops : public Module{
    private:
        void wait_for_homed(char axes_to_move);

    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);

        StepperMotor* steppers[3];
        Pin           pins[6];
        double  slow_rates[3];
        double  fast_rates[3];
        double  trim[3];
        unsigned int  retract_steps[3];
        unsigned int  debounce_count;
        bool home_direction[3];
        double homing_position[3];
        char status;
};

















#endif
