/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Homer.h"
#include "Kernel.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Endstop.h"
#include "Gcode.h"
#include "CartesianAxis.h"
#include "arm_solutions/BaseSolution.h"

#include <math.h>

#define homing_order_checksum CHECKSUM("homing_order")

Homer::Homer()
{
}

void Homer::on_module_loaded()
{
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_CONFIG_RELOAD);

    on_config_reload(this);
}

void Homer::on_config_reload(void*)
{
    homing_order = THEKERNEL->config->value(homing_order_checksum)->by_default("ABC,XY,Z")->as_string();
}

// Start homing sequences by response to GCode commands
void Homer::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (gcode->has_g && gcode->g == 28)
    {
        gcode->mark_as_taken();
        // G28 is received, we have homing to do

        // wait for the queue to be empty
        THEKERNEL->conveyor->wait_for_empty_queue();

        home_set(assemble_set_from_gcode(gcode));
    }
}

string Homer::assemble_set_from_gcode(Gcode* gcode)
{
    string set = homing_order;
    bool config_order = true;

    for (int i = 0; i < 3; i++)
    {
        if (gcode->has_letter('A' + i))
        {
            if (config_order)
                set.clear();
            config_order = false;

            set += 'A' + i;

            // if user sends G28 A1, home to max
            if (gcode->get_value('A' + i) > 0.0F)
                set += '+';
        }
        if (gcode->has_letter('X' + i))
        {
            if (config_order)
                set.clear();
            config_order = false;

            set += 'X' + i;

            // if user sends G28 X1, home to max
            if (gcode->get_value('X' + i) > 0.0F)
                set += '+';
        }
    }

    // ensure last item gets homed
    set += ',';

    return set;
}

void Homer::home_set(string set)
{
    struct homing_info
    {
        Actuator* actuator;

        struct {
            bool home_to_max :1;
            bool is_axis     :1;
        };
    };

    std::vector<struct homing_info> actuators;
    actuators.reserve(3);

    for (auto i = set.begin(); i != set.end(); i++)
    {
        if (*i >= 'A' && *i <= 'C')
        {
            actuators.resize(actuators.size() + 1);
            actuators.back().actuator = THEKERNEL->robot->actuators[*i - 'A'];
            actuators.back().is_axis = false;
        }
        else if (*i >= 'X' && *i <= 'Z')
        {
            actuators.resize(actuators.size() + 1);
            actuators.back().actuator = &THEKERNEL->robot->axes[*i - 'X'];
            actuators.back().is_axis = true;
        }
        else if (*i == '+')
            actuators.back().home_to_max = true;
        else if (*i == '-')
            actuators.back().home_to_max = false;
        else if (*i == ',')
        {
            // TODO: home this set

            THEKERNEL->serial->printf("Homing Set:\n");
            for (auto j = actuators.begin(); j != actuators.end(); j++)
            {
                THEKERNEL->serial->printf("\t%p: %c to %s\n", j->actuator, j->actuator->designator, j->home_to_max?"MAX":"MIN");
            }

            actuators.clear();
        }
    }
}
