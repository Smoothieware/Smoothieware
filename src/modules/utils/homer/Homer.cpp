#include "Homer.h"

#include "Endstop.h"
#include "Endstops.h"
#include "Kernel.h"
#include "Conveyor.h"
#include "StepperMotor.h"

#include <vector>

Homer::Homer()
{
}

void Homer::on_module_loaded()
{
    register_for_event(ON_GCODE_RECEIVED);
}

void Homer::on_gcode_received(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);

    if (gcode->has_g && gcode->g == 28) // G28 - Home axes
    {
        gcode->mark_as_taken();

        // wait for queue flush so we can safely issue a series of Blocks
        THEKERNEL->conveyor->wait_for_empty_queue();

        // if the arm solution has some magic, don't do normal homing
//             if (THEKERNEL->robot->arm_solution->special_homing())
//                 return;

        /*
            * The plan:
            *
            * 1) gather per-actuator endstops- alpha_max, etc
            * 2) find nominal_rate given final_rate = minimum_planner_speed and decelerate_distance = user-supplied stopping_distance
            * 3) issue a block with actuator_length per actuator distance
            * 4) when we detect an endstop, reach inside the block and set decelerate_steps=0 to force deceleration now
            * 5) TODO: find how to make the block finish when it has decelerated
            *
            * 6) gather per-axis endstops- x_max, etc
            * 7) as per 2) above
            * 8) issue a block with axis_length per axis distance
            * 9) as per 4) above
            * 10) as per 5) above
            *
            */

        float target[3]   = {0.0F, 0.0F, 0.0F};
        float unit_vec[3] = {0.0F, 0.0F, 0.0F};
        float feed_rate;
        float distance;
        float ud = 0.0F;

        bool  prev_check[3];

        bool  all_axes = true;
        bool  axis_flag[6] = { false, false, false, false, false, false }; // A,B,C,X,Y,Z

        for (int i = 0; i < 3; i++)
        {
            if (gcode->has_letter('A' + i)) // A,B,C
            {
                all_axes = false;
                axis_flag[i] = true;
            }

            if (gcode->has_letter('X' + i)) // X,Y,Z
            {
                all_axes = false;
                axis_flag[i + 3] = true;
            }
        }

        if (all_axes)
        {
            for (int i = 0; i < 6; i++)
                axis_flag[i] = true;
        }

        // we need to 1) find which axes have endstops and 2) fudge some numbers so Planner does what we want
        distance =  500.0F; // TODO: gather user-setting for per-axis lengths, find largest one
        feed_rate =   5.0F; // TODO: 1) gather user-setting for homing rate, 2) calculate homing rate given stopping distance and acceleration

        for (int i = 0; i < 3; i++)
        {
            prev_check[i] = THEKERNEL->robot->actuators[i]->check_endstops;

            if (axis_flag[i])
            {
                // If both endstops are available, and user specifies G28 A0, prefer min. if user specifies G28 A1, prefer max
                if (THEKERNEL->robot->actuators[i]->min_stop && THEKERNEL->robot->actuators[i]->max_stop && gcode->has_letter('A' + i))
                {
                    if (gcode->get_value('A' + i) <= 0)
                        target[i] = -distance; // use min stop
                    else
                        target[i] =  distance; // use max stop
                }
                else if (THEKERNEL->robot->actuators[i]->min_stop)
                    target[i] = -distance;
                else if (THEKERNEL->robot->actuators[i]->max_stop)
                    target[i] =  distance;

                if (target[i] != 0.0F)
                {
                    THEKERNEL->robot->actuators[i]->check_endstops = true;
                    THEKERNEL->robot->actuators[i]->change_last_milestone(0.0F);
                    ud += fabs(target[i]);
                }
                else
                    axis_flag[i] = false;
            }
        }

        if (ud != 0.0F)
        {
            THEKERNEL->planner->append_block(target, feed_rate, distance, unit_vec);
            THEKERNEL->conveyor->ensure_running();

            THEKERNEL->conveyor->wait_for_empty_queue();

            for (int i = 0; i < 3; i++)
            {
                if (axis_flag[i])
                {
                    THEKERNEL->robot->actuators[i]->check_endstops = prev_check[i];
                    if (target[i] > 0.0F)
                        THEKERNEL->robot->actuators[i]->change_last_milestone(THEKERNEL->robot->actuators[i]->max_stop->position);
                    else if (target[i] < 0.0F)
                        THEKERNEL->robot->actuators[i]->change_last_milestone(THEKERNEL->robot->actuators[i]->min_stop->position);
                }
            }
        }

        // TODO: forward kinematics, actuator position -> cartesian so future blocks don't get screwed up

        // TODO: now home XYZ
    }
}
