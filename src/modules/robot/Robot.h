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
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "../communication/utils/Gcode.h"
#include "arm_solutions/BaseSolution.h"
#include "Planner.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"


#define default_seek_rate_checksum             47357
#define default_feed_rate_checksum             6633
#define mm_per_line_segment_checksum           30176
#define mm_per_arc_segment_checksum            15470
#define arc_correction_checksum                5074
#define x_axis_max_speed_checksum              64935
#define y_axis_max_speed_checksum              3752
#define z_axis_max_speed_checksum              7849

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2

#define MOTION_MODE_SEEK 0 // G0
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2 // G2
#define MOTION_MODE_CCW_ARC 3 // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS 2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1



class Robot : public Module {
    public:
        Robot();
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);

    private:
        void execute_gcode(Gcode* gcode);
        void append_milestone( double target[], double feed_rate);
        void append_line( Gcode* gcode, double target[], double feed_rate);
        //void append_arc(double theta_start, double angular_travel, double radius, double depth, double rate);
        void append_arc( Gcode* gcode, double target[], double offset[], double radius, bool is_clockwise );


        void compute_arc(Gcode* gcode, double offset[], double target[]);
        double to_millimeters(double value);
        double theta(double x, double y);
        void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);

        double current_position[3];                           // Current position, in millimeters
        double last_milestone[3];                             // Last position, in millimeters
        bool inch_mode;                                       // true for inch mode, false for millimeter mode ( default )
        bool absolute_mode;                                   // true for absolute mode ( default ), false for relative mode
        int8_t motion_mode;                                   // Motion mode for the current received Gcode
        double seek_rate;                                     // Current rate for seeking moves ( mm/s )
        double feed_rate;                                     // Current rate for feeding moves ( mm/s )
        uint8_t plane_axis_0, plane_axis_1, plane_axis_2;     // Current plane ( XY, XZ, YZ )
        BaseSolution* arm_solution;                           // Selected Arm solution ( millimeters to step calculation )
        double mm_per_line_segment;                           // Setting : Used to split lines into segments
        double mm_per_arc_segment;                            // Setting : Used to split arcs into segmentrs

        // Number of arc generation iterations by small angle approximation before exact arc trajectory
        // correction. This parameter maybe decreased if there are issues with the accuracy of the arc
        // generations. In general, the default value is more than enough for the intended CNC applications
        // of grbl, and should be on the order or greater than the size of the buffer to help with the
        // computational efficiency of generating arcs.
        int arc_correction;                                   // Setting : how often to rectify arc computation
        double max_speeds[3];                                 // Setting : max allowable speed in mm/m for each axis

    // Used by Stepper
    public:
        Pin* alpha_step_pin;
        Pin* beta_step_pin;
        Pin* gamma_step_pin;
        Pin* alpha_dir_pin;
        Pin* beta_dir_pin;
        Pin* gamma_dir_pin;
        Pin* alpha_en_pin;
        Pin* beta_en_pin;
        Pin* gamma_en_pin;

        StepperMotor* alpha_stepper_motor;
        StepperMotor* beta_stepper_motor;
        StepperMotor* gamma_stepper_motor;

        double seconds_per_minute;                            // for realtime speed change
};

#endif
