/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BLOCK_H
#define BLOCK_H
#include "libs/Module.h"
#include "libs/Kernel.h"
using namespace std;
#include <string>
#include <vector>

#include "../communication/utils/Gcode.h"
#include "Planner.h"
class Planner;
class Conveyor;

float max_allowable_speed( float acceleration, float target_velocity, float distance);

class Block {
    public:
        Block();
        void calculate_trapezoid( float entry_speed, float exit_speed );
        float estimate_acceleration_distance( float initial_rate, float target_rate, float acceleration );
        float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance);
        float get_duration_left(unsigned int already_taken_steps);

        float reverse_pass(float exit_speed);
        float forward_pass(float next_entry_speed);

        float max_exit_speed();

        void debug();

        void append_gcode(Gcode* gcode);

        void take();
        void release();

        void ready();

        void clear();

        void begin();

        vector<Gcode> gcodes;

        unsigned int   steps[3];           // Number of steps for each axis for this block

        unsigned int   steps_event_count;  // Steps for the longest axis
        unsigned int   accelerate_until;   // Stop accelerating after this number of steps
        unsigned int   decelerate_after;   // Start decelerating after this number of steps

        unsigned int   initial_rate;       // Initial speed in steps per second
        unsigned int   nominal_rate;       // Nominal speed in steps per second
        unsigned int   final_rate;         // Final   speed in steps per second

        float          nominal_speed;      // Nominal speed in mm per second
        float          millimeters;        // Distance for this move (mm)
        float          entry_speed;        // entry speed (mm/s)
        float          exit_speed;         // exit speed (mm/s) - stores the exit speed that the trapezoid generator is targeting when a block is currently active
        float          rate_delta;         // Nomber of steps to add to the speed for each acceleration tick
        float          max_entry_speed;    // highest allowable entry speed given deceleration profile and permissible exit speed

        int8_t         times_taken;        // A block can be "taken" by any number of modules, and the next block is not moved to until all the modules have "released" it. This value serves as a tracker.

        union {
            uint8_t       flags;
            struct {
                uint8_t direction_bits      :3; // Direction for each axis in bit form, relative to the direction port's mask
                bool    recalculate_flag    :1; // Planner flag to recalculate trapezoids on entry junction
                bool    nominal_length_flag :1; // Planner flag for nominal speed always reached
                bool    is_ready            :1;
                bool    finish_early        :1; // assert if we want the block to end NOW rather than finish all steps
            };
        };
};

#endif
