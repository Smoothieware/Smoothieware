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
#include "arm_solutions/BaseSolution.h"

#include <math.h>

Homer::Homer()
{
}

void Homer::on_module_loaded()
{
    register_for_event(ON_GCODE_RECEIVED);
}

void Homer::home_actuators(Gcode* gcode)
{
    uint8_t actuators[3] = {NO_HOME, NO_HOME, NO_HOME};

    bool home_all = true;
    for (int i = 0; i < 3; i++)
    {
        if (gcode->has_letter('A' + i) || gcode->has_letter('X' + i))
            home_all = false;
    }

    // work out which actuators are selected for homing
    for (int i = 0; i < 3; i++)
    {
        if (home_all || gcode->has_letter('A' + i))
        {
            Actuator* act = THEKERNEL->robot->actuators[i];

            if (act->min_stop && act->max_stop)
            {
                if (gcode->get_value('A' + i) > 0.0F)
                    actuators[i] = HOME_TO_MAX;
                else
                    actuators[i] = HOME_TO_MIN;
            }
            else if (act->min_stop)
                actuators[i] = HOME_TO_MIN;
            else if (act->max_stop)
                actuators[i] = HOME_TO_MAX;
            else
                actuators[i] = NO_HOME;
        }
    }

    if (THEKERNEL->robot->arm_solution->confirm_homing_set(actuators) == false)
    {
        // arm solution says illegal combination
        gcode->stream->printf("Error: Illegal actuator set while homing!\n");
        return;
    }

    if ((actuators[0] == 0) && (actuators[1] == 0) && (actuators[2] == 0))
        return;

    float distance = 0.0F;
    float dest_vector[3] = {0.0F, 0.0F, 0.0F};

    for (int i = 0; i < 3; i++)
    {
        if (actuators[i])
        {
            Actuator* act = THEKERNEL->robot->actuators[i];

            // work out actuator travel from soft limits
            dest_vector[i] = act->soft_max - act->soft_min;

            // if soft limits are NAN, use a default of 500mm
            if (isnan(dest_vector[i]))
                dest_vector[i] = 500.0F;

            // find actuator that moves greatest distance
            if (dest_vector[i] > distance)
                distance = dest_vector[i];

            // move in negative direction if homing to min
            if (actuators[i] == HOME_TO_MIN)
                dest_vector[i] *= -1.0F;

            // add current position, so we can work out position where endstop triggered
            dest_vector[i] += act->last_milestone;
        }
    }

    if (distance > 0.0F)
    {
        float unit_vec[3] = {0.0F, 0.0F, 0.0F};
        bool check[3];

        THEKERNEL->planner->append_block(dest_vector, 10.0F, distance, unit_vec);

        // set all active endstops to be checked
        for (int i = 0; i < 3; i++)
        {
            if (THEKERNEL->robot->actuators[i]->active_stop)
            {
                check[i] = THEKERNEL->robot->actuators[i]->active_stop->check;
                THEKERNEL->robot->actuators[i]->active_stop->check = true;
            }
        }

        // wait for move to complete
        THEKERNEL->conveyor->wait_for_empty_queue();

        // update actuator positions
        for (int i = 0; i < 3; i++)
        {
            if (THEKERNEL->robot->actuators[i]->active_stop)
            {
                THEKERNEL->robot->actuators[i]->active_stop->check = check[i];
                // TODO: cache or report endstop trigger position
                THEKERNEL->robot->actuators[i]->change_last_milestone(THEKERNEL->robot->actuators[i]->active_stop->position);
            }
        }
    }

    // reuse dest_vector as current actuator position
    for (int i = 0; i < 3; i++)
        dest_vector[i] = THEKERNEL->robot->actuators[i]->last_milestone;

    float cartesian[3];

    // perform forward kinematics to find cartesian position
    THEKERNEL->robot->arm_solution->actuator_to_cartesian(dest_vector, cartesian);

    // update axis positions
    for (int i = 0; i < 3; i++)
        THEKERNEL->robot->axes[i].change_last_milestone(cartesian[i]);
}

void Homer::home_axes(Gcode* gcode)
{
    uint8_t axes[3] = {NO_HOME, NO_HOME, NO_HOME};

    // work out which axes are selected for homing
    bool home_all = true;
    for (int i = 0; i < 3; i++)
    {
        if (gcode->has_letter('A' + i) || gcode->has_letter('X' + i))
            home_all = false;
    }

    for (int i = 0; i < 3; i++)
    {
        if (home_all || gcode->has_letter('X' + i))
        {
            Actuator* act = &THEKERNEL->robot->axes[i];

            if (act->min_stop && act->max_stop)
            {
                if (gcode->get_value('X' + i) > 0.0F)
                    axes[i] = HOME_TO_MAX;
                else
                    axes[i] = HOME_TO_MIN;
            }
            else if (act->min_stop)
                axes[i] = HOME_TO_MIN;
            else if (act->max_stop)
                axes[i] = HOME_TO_MAX;
            else
                axes[i] = NO_HOME;
        }
    }

    if ((axes[0] == 0) && (axes[1] == 0) && (axes[2] == 0))
        return;

    while ((axes[0] != 0) || (axes[1] != 0) || (axes[2] != 0))
    {
        float distance = 0.0F;
        float dest_vector[3] = {0.0F, 0.0F, 0.0F};

        for (int i = 0; i < 3; i++)
        {
            if (axes[i])
            {
                Actuator* act = &THEKERNEL->robot->axes[i];

                // work out actuator travel from soft limits
                dest_vector[i] = act->soft_max - act->soft_min;

                // if soft limits are NAN, use a default of 500mm
                if (isnan(dest_vector[i]))
                    dest_vector[i] = 500.0F;

                // find actuator that moves greatest distance
                if (dest_vector[i] > distance)
                    distance = dest_vector[i];

                // move in negative direction if homing to min
                if (axes[i] == HOME_TO_MIN)
                    dest_vector[i] *= -1.0F;

                // add current position, so we can work out position where endstop triggered
                dest_vector[i] += act->last_milestone;
            }
        }

        if (distance > 0.0F)
        {
            float unit_vec[3] = {0.0F, 0.0F, 0.0F};
            float act_vector[3];

            THEKERNEL->robot->arm_solution->cartesian_to_actuator(dest_vector, act_vector);

            THEKERNEL->planner->append_block(act_vector, 10.0F, distance, unit_vec);

            // set up endstops
            for (int i = 0; i < 3; i++)
            {
                Actuator* axis = &THEKERNEL->robot->axes[i];
                if (axes[i] == HOME_TO_MAX)
                    axis->active_stop = axis->max_stop;
                else if (axes[i] == HOME_TO_MIN)
                    axis->active_stop = axis->min_stop;
            }

            // wait for move to complete
            while (THEKERNEL->conveyor->queue.is_empty() == false)
            {
                int triggered = -1;
                for (int i = 0; i < 3; i++)
                {
                    Actuator* axis = &THEKERNEL->robot->axes[i];
                    if ((axes[i] != NO_HOME) && axis->active_stop && axis->active_stop->asserted())
                    {
                        // endstop triggered!
                        triggered = i;
                        break;
                    }
                }
                if (triggered >= 0)
                {
                    // stop all actuators
                    // TODO: decelerate instead of stopping dead
                    for (int i = 0; i < 3; i++)
                        THEKERNEL->robot->actuators[i]->move(0, 0);

                    axes[triggered] = NO_HOME;

                    Actuator* axis = &THEKERNEL->robot->axes[triggered];

                    THEKERNEL->conveyor->wait_for_empty_queue();

                    axis->change_last_milestone(axis->active_stop->position);
                    axis->active_stop = NULL;
                }
            }
        }
    }

    // update positions
    float cartesian[3], actuator[3];

    for (int i = 0; i < 3; i++)
        cartesian[i] = THEKERNEL->robot->axes[i].last_milestone;

    THEKERNEL->robot->arm_solution->cartesian_to_actuator(cartesian, actuator);

    for (int i = 0; i < 3; i++)
        THEKERNEL->robot->actuators[i]->change_last_milestone(actuator[i]);
}

// Start homing sequences by response to GCode commands
void Homer::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (gcode->has_g && gcode->g == 28)
    {
        gcode->mark_as_taken();
        // G28 is received, we have homing to do

        // First wait for the queue to be empty
        THEKERNEL->conveyor->wait_for_empty_queue();

        home_actuators(gcode);
        home_axes(gcode);
    }
}
