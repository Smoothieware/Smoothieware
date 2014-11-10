/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PLANNER_H
#define PLANNER_H

class Block;

class Planner
{
public:
    Planner();
    void append_block( float target[], float rate_mm_s, float distance, float unit_vec[] );
    float max_allowable_speed( float acceleration, float target_velocity, float distance);
    void recalculate();
    Block *get_current_block();
    void cleanup_queue();
    float get_acceleration() const { return acceleration; }
    float get_z_acceleration() const { return z_acceleration > 0.0F ? z_acceleration : acceleration; }

    friend class Robot; // for acceleration, junction deviation, minimum_planner_speed

private:
    void config_load();
    float previous_unit_vec[3];
    float acceleration;          // Setting
    float z_acceleration;        // Setting
    float junction_deviation;    // Setting
    float z_junction_deviation;  // Setting
    float minimum_planner_speed; // Setting
};



#endif
