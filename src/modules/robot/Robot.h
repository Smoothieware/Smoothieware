/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ROBOT_H
#define ROBOT_H

#include <string>
using std::string;
#include <string.h>
#include <functional>
#include <stack>

#include "libs/Module.h"

class Gcode;
class BaseSolution;
class StepperMotor;

class Robot : public Module {
    public:
        Robot();
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_halt(void *arg);

        void reset_axis_position(float position, int axis);
        void reset_axis_position(float x, float y, float z);
        void reset_position_from_current_actuator_position();
        void get_axis_position(float position[]);
        float to_millimeters(float value);
        float from_millimeters(float value);
        float get_seconds_per_minute() const { return seconds_per_minute; }
        float get_z_maxfeedrate() const { return this->max_speeds[2]; }
        void setToolOffset(const float offset[3]);
        float get_feed_rate() const { return feed_rate; }

        BaseSolution* arm_solution;                           // Selected Arm solution ( millimeters to step calculation )

        // gets accessed by Panel, Endstops, ZProbe
        std::vector<StepperMotor*> actuators;

        // set by a leveling strategy to transform the target of a move according to the current plan
        std::function<void(float[3])> compensationTransform;

        struct {
            bool inch_mode:1;                                 // true for inch mode, false for millimeter mode ( default )
            bool absolute_mode:1;                             // true for absolute mode ( default ), false for relative mode
        };

    private:
        void distance_in_gcode_is_known(Gcode* gcode);
        void append_milestone( Gcode *gcode, float target[], float rate_mm_s);
        void append_line( Gcode* gcode, float target[], float rate_mm_s);
        //void append_arc(float theta_start, float angular_travel, float radius, float depth, float rate);
        void append_arc( Gcode* gcode, float target[], float offset[], float radius, bool is_clockwise );


        void compute_arc(Gcode* gcode, float offset[], float target[]);

        float theta(float x, float y);
        void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);
        void clearToolOffset();
        void check_max_actuator_speeds();

        typedef std::tuple<float, float, bool> saved_state_t; // save current feedrate and absolute mode
        std::stack<saved_state_t> state_stack;               // saves state from M120
        float last_milestone[3];                             // Last position, in millimeters
        float transformed_last_milestone[3];                 // Last transformed position
        int8_t motion_mode;                                  // Motion mode for the current received Gcode
        uint8_t plane_axis_0, plane_axis_1, plane_axis_2;    // Current plane ( XY, XZ, YZ )
        float seek_rate;                                     // Current rate for seeking moves ( mm/s )
        float feed_rate;                                     // Current rate for feeding moves ( mm/s )
        float mm_per_line_segment;                           // Setting : Used to split lines into segments
        float mm_per_arc_segment;                            // Setting : Used to split arcs into segmentrs
        float delta_segments_per_second;                     // Setting : Used to split lines into segments for delta based on speed
        float seconds_per_minute;                            // for realtime speed change

        // Number of arc generation iterations by small angle approximation before exact arc trajectory
        // correction. This parameter maybe decreased if there are issues with the accuracy of the arc
        // generations. In general, the default value is more than enough for the intended CNC applications
        // of grbl, and should be on the order or greater than the size of the buffer to help with the
        // computational efficiency of generating arcs.
        int arc_correction;                                   // Setting : how often to rectify arc computation
        float max_speeds[3];                                 // Setting : max allowable speed in mm/m for each axis

        float toolOffset[3];

        // Used by Stepper, Planner
        friend class Planner;
        friend class Stepper;

        StepperMotor* alpha_stepper_motor;
        StepperMotor* beta_stepper_motor;
        StepperMotor* gamma_stepper_motor;

        struct {
            bool halted:1;
        };
};

// Convert from inches to millimeters ( our internal storage unit ) if needed
inline float Robot::to_millimeters( float value ){
    return this->inch_mode ? value * 25.4 : value;
}
inline float Robot::from_millimeters( float value){
    return this->inch_mode ? value/25.4 : value;
}
inline void Robot::get_axis_position(float position[]){
    memcpy(position, this->last_milestone, sizeof(float)*3 );
}

#endif
