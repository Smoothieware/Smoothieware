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
        void calculate_trapezoid( float entry_factor, float exit_factor );
        float estimate_acceleration_distance( float initial_rate, float target_rate, float acceleration );
        float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance);
        void reverse_pass(Block* previous);
        void forward_pass(Block* next);
        void debug();
        void append_gcode(Gcode* gcode);
        void pop_and_execute_gcode();
        float get_duration_left(unsigned int already_taken_steps);
        void take();
        void release();
        void ready();

        vector<std::string> commands;
        vector<float> travel_distances;
        vector<Gcode> gcodes;

        unsigned int   steps[3];           // Number of steps for each axis for this block
        unsigned int   steps_event_count;  // Steps for the longest axis
        unsigned int   nominal_rate;       // Nominal rate in steps per minute
        float          nominal_speed;      // Nominal speed in mm per minute
        float          millimeters;        // Distance for this move
        float         entry_speed;
        float          rate_delta;         // Nomber of steps to add to the speed for each acceleration tick
        unsigned int   initial_rate;       // Initial speed in steps per minute
        unsigned int   final_rate;         // Final speed in steps per minute
        unsigned int   accelerate_until;   // Stop accelerating after this number of steps
        unsigned int   decelerate_after;   // Start decelerating after this number of steps
        unsigned int   direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask


        uint8_t recalculate_flag; // Planner flag to recalculate trapezoids on entry junction
        uint8_t nominal_length_flag; // Planner flag for nominal speed always reached

        float max_entry_speed;
        Planner* planner;
        Conveyor*  conveyor;
        
        bool is_ready;

        short times_taken;    // A block can be "taken" by any number of modules, and the next block is not moved to until all the modules have "released" it. This value serves as a tracker.

};











#endif
