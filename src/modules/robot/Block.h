/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BLOCK_H
#define BLOCK_H

#include <vector>
#include <bitset>
#include "ActuatorCoordinates.h"

class Gcode;

class Block {
    public:
        Block();
        void calculate_trapezoid( float entry_speed, float exit_speed );
        float max_allowable_speed( float acceleration, float target_velocity, float distance);

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);

        float max_exit_speed();

        void debug();

       // void append_gcode(Gcode* gcode);

        void release();

        void ready();

        void clear();

        void begin();

        //std::vector<Gcode> gcodes;

        std::array<uint32_t, k_max_actuators> steps; // Number of steps for each axis for this block
        uint32_t steps_event_count;  // Steps for the longest axis
        float nominal_rate;       // Nominal rate in steps per second
        float nominal_speed;      // Nominal speed in mm per second
        float millimeters;        // Distance for this move
        float entry_speed;
        float exit_speed;
        float acceleration;       // the acceleratoin for this block
        float initial_rate;       // Initial rate in steps per second
        uint32_t accelerate_until;   // Stop accelerating after this number of ticks
        uint32_t decelerate_after;   // Start decelerating after this number of ticks
        float maximum_rate;

        float acceleration_per_tick{0};
        float deceleration_per_tick {0};
        uint32_t total_move_ticks{0};

        float max_entry_speed;

        std::bitset<k_max_actuators> direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask
        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction
            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached
            bool is_ready:1;
        };
};


#endif
