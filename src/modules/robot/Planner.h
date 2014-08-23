/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PLANNER_H
#define PLANNER_H

class Block;

namespace Planner
{
    void init();
    void append_block( float target[], float rate_mm_s, float distance, float unit_vec[] );
    float max_allowable_speed( float acceleration, float target_velocity, float distance);
    void recalculate();
    Block *get_current_block();
    void cleanup_queue();
    
    float get_acceleration();
    float get_junction_deviation();
    float get_minimum_planner_speed();
    float get_z_acceleration();

    void set_acceleration(float acceleration);
    void set_junction_deviation(float deviation);
    void set_minimum_planner_speed(float speed);
    void set_z_acceleration(float acceleration);
};



#endif
