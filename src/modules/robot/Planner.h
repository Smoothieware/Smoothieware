/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include "libs/RingBuffer.h"
#include "../communication/utils/Gcode.h"
#include "Block.h"

#define acceleration_checksum       25326  
#define max_jerk_checksum           61012 
#define junction_deviation_checksum 6035 

// TODO: Get from config
#define MINIMUM_PLANNER_SPEED 0.0
using namespace std;


class Planner : public Module {
    public:
        Planner();
        void append_block( int target[], double feed_rate, double distance, double deltas[] );
        double max_allowable_speed( double acceleration, double target_velocity, double distance);
        void recalculate();
        void reverse_pass();
        void forward_pass();
        void recalculate_trapezoids();
        void dump_queue();
        Block* get_current_block(); 
        void cleanup_queue();
        void on_module_loaded();
        void on_config_reload(void* argument);

        int position[3];              // Current position, in steps
        double previous_unit_vec[3];
        Block last_deleted_block;     // Item -1 in the queue, TODO: Grbl does not need this, but Smoothie won't work without it, we are probably doing something wrong
        bool has_deleted_block;       // Flag for above value
        float previous_nominal_speed;

        double acceleration;          // Setting
        double max_jerk;              // Setting
        double junction_deviation;    // Setting

};



#endif
