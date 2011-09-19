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

class Block {
    public:
        Block();
        double compute_factor_for_safe_speed();
        void calculate_trapezoid( double entry_factor, double exit_factor );
        double estimate_acceleration_distance( double initial_rate, double target_rate, double acceleration );
        double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance); 
        void reverse_pass(Block* previous, Block* next);
        void forward_pass(Block* previous, Block* next);
        void debug(Kernel* kernel);
        void append_gcode(Gcode* gcode);
        void pop_and_execute_gcode(Kernel* &kernel);
        double get_duration_left(unsigned int already_taken_steps);
        vector<std::string> commands;

        unsigned short steps[3];           // Number of steps for each axis for this block
        float          speeds[3];          // Speeds for each axis, used in the planning process
        unsigned short steps_event_count;  // Steps for the longest axis
        unsigned int   nominal_rate;       // Nominal rate in steps per minute
        float          nominal_speed;      // Nominal speed in mm per minute
        float          millimeters;        // Distance for this move
        float          entry_factor;       // Starting speed for the block
        unsigned int rate_delta;           // Nomber of steps to add to the speed for each acceleration tick
        unsigned int   initial_rate;       // Initial speed in steps per minute
        unsigned int   final_rate;         // Final speed in steps per minute
        unsigned short accelerate_until;   // Stop accelerating after this number of steps
        unsigned short decelerate_after;   // Start decelerating after this number of steps
        unsigned int   direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask

        Planner* planner;
};











#endif
