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

#include <map>

using std::map;

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

        confirm_set(assemble_set_from_gcode(gcode));
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

void Homer::confirm_set(string set)
{
    vector<struct homing_info> actuators;
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
            for (auto i = actuators.begin(); i != actuators.end();)
            {
                if ((i->actuator->max_stop == NULL) && (i->actuator->min_stop == NULL))
                {
                    actuators.erase(i);
                    i = actuators.begin();
                    continue;
                }

                if (i->home_to_max && (i->actuator->max_stop == NULL) && (i->actuator->min_stop))
                    i->home_to_max = false;
                else if ((i->home_to_max == false) && (i->actuator->max_stop) && (i->actuator->min_stop == NULL))
                    i->home_to_max = true;

                i++;
            }

            if (actuators.size())
            {
                home_set(actuators);
            }

            actuators.clear();
        }
    }
}

void Homer::home_set(vector<struct homing_info>& actuators)
{
    map<homing_info*, float> motion_vector;

    THEKERNEL->serial->printf("Homing Set:\n");

    for (homing_info& a : actuators)
    {
        a.state = HOMING_STATE_FAST_HOME;
        motion_vector[&a] = INFINITY;

        THEKERNEL->serial->printf("\t%p: %c to %s\n", a.actuator, a.actuator->designator, a.home_to_max?"MAX":"MIN");
    }

    while (actuators.size())
    {
        for (homing_info& a : actuators)
        {
            float vec = 0.0F;
            enum homing_states new_state = a.state;

            switch (a.state)
            {
                /*
                 * STATE ACTIONS
                 */
                case HOMING_STATE_FAST_HOME:
                {
                    vec = 1.0F;
                    if (a.actuator->active_stop->asserted())
                        new_state = HOMING_STATE_FAST_DEASSERT_ENDSTOP;
                    break;
                }
                case HOMING_STATE_FAST_DEASSERT_ENDSTOP:
                {
                    vec = -1.0F;
                    if (a.actuator->active_stop->asserted())
                        new_state = HOMING_STATE_FAST_RETRACT;
                    break;
                }
                case HOMING_STATE_FAST_RETRACT:
                {
                    vec = -1.0F;
                    // TODO: stop when we've moved far enough
                    break;
                }

                case HOMING_STATE_SLOW_HOME:
                {
                    vec = 0.1F;
                    if (a.actuator->active_stop->asserted())
                        new_state = HOMING_STATE_SLOW_DEASSERT_ENDSTOP;
                    break;
                }
                case HOMING_STATE_SLOW_DEASSERT_ENDSTOP:
                {
                    vec = -0.1F;
                    if (a.actuator->active_stop->asserted())
                        new_state = HOMING_STATE_SLOW_RETRACT;
                    break;
                }
                case HOMING_STATE_SLOW_RETRACT:
                {
                    vec = -0.1F;
                    // TODO: stop when we've moved far enough
                    break;
                }

                case HOMING_STATE_POST_HOME_MOVE:
                {
                    // TODO
                    vec = 0.0F;
                    break;
                }
                case HOMING_STATE_DONE:
                {
                    vec = 0.0F;
                    break;
                }

                default:
                    new_state = HOMING_STATE_DONE;
                    break;
            }

            if (vec != motion_vector[&a])
            {
                // TODO: update continuous motion vector in Robot or Stepper
                if (a.home_to_max)
                    vec = -vec;

                // sanity check
                if (isinf(vec))
                {
                    a.state = HOMING_STATE_DONE;
                    motion_vector[&a] = 0.0F;
                }
                else
                    motion_vector[&a] = vec;
            }

            if (new_state != a.state)
            {
                /*
                 * STATE TRANSITIONS
                 */
                switch (new_state)
                {
                    case HOMING_STATE_FAST_DEASSERT_ENDSTOP:
                    case HOMING_STATE_SLOW_DEASSERT_ENDSTOP:
                    {
                        // select appropriate endstop- StepperMotor will choose the wrong one since we're moving away from it
                        a.actuator->active_stop = (a.home_to_max) ? a.actuator->max_stop : a.actuator->min_stop;
                        break;
                    }
                    case HOMING_STATE_POST_HOME_MOVE:
                    {
                        // TODO: flush queue, insert a regular move, wait for it to complete then continue homing
                        THEKERNEL->conveyor->flush_queue();

                        break;
                    }
                    default:
                        break;
                }

                a.state = new_state;
            }
        }

        for (auto a = actuators.begin(); a != actuators.end(); a++)
        {
            if (a->state == HOMING_STATE_DONE)
            {
                actuators.erase(a);
                a = actuators.begin();
            }
        }
    }
}
