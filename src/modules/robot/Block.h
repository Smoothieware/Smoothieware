/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <vector>
#include <bitset>
#include "ActuatorCoordinates.h"

class Block {
    public:
        Block();
        void calculate_trapezoid( float entry_speed, float exit_speed );
        float max_allowable_speed( float acceleration, float target_velocity, float distance);

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);
        float max_exit_speed();
        void debug() const;
        void ready() { is_ready= true; }
        void clear();
        void prepare();

        std::array<uint32_t, k_max_actuators> steps; // Number of steps for each axis for this block
        uint32_t steps_event_count;  // Steps for the longest axis
        float nominal_rate;       // Nominal rate in steps per second
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float entry_speed;
        float exit_speed;
        float acceleration;       // the acceleration for this block
        float initial_rate;       // Initial rate in steps per second
        float maximum_rate;

        float acceleration_per_tick{0};
        float deceleration_per_tick {0};

        float max_entry_speed;

        // this is tick info needed for this block. applies to all motors
        uint32_t accelerate_until;
        uint32_t decelerate_after;
        uint32_t total_move_ticks;
        std::bitset<k_max_actuators> direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask

        // this is the data needed to determine when each motor needs to be issued a step
        using tickinfo_t= struct {
            int32_t steps_per_tick; // 2.30 fixed point
            int32_t counter; // 2.30 fixed point
            int32_t acceleration_change; // 2.30 fixed point signed
            int32_t deceleration_change; // 2.30 fixed point
            int32_t plateau_rate; // 2.30 fixed point
            uint32_t steps_to_move;
            uint32_t step_count;
            uint32_t next_accel_event;
        };

        // need info for each active motor
        //std::array<tickinfo_t, k_max_actuators> tick_info;
        std::vector<tickinfo_t> tick_info;
        static uint8_t n_actuators;

        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction
            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached
            bool is_ready:1;
            bool primary_axis:1;                 // set if this move is a primary axis
            volatile bool is_ticking:1;          // set when this block is being actively ticked by the stepticker
            volatile bool locked:1;              // set to true when the critical data is being updated, stepticker will have to skip if this is set
        };
};
