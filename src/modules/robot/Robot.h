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
#include "ActuatorCoordinates.h"

class Gcode;
class BaseSolution;
class StepperMotor;

// 9 WCS offsets
#define MAX_WCS 9UL

class Robot : public Module {
    public:
        using wcs_t= std::tuple<float, float, float>;
        Robot();
        void on_module_loaded();
        void on_gcode_received(void* argument);

        void reset_axis_position(float position, int axis);
        void reset_axis_position(float x, float y, float z);
        void reset_actuator_position(const ActuatorCoordinates &ac);
        void reset_position_from_current_actuator_position();
        float get_seconds_per_minute() const { return seconds_per_minute; }
        float get_z_maxfeedrate() const { return this->max_speeds[2]; }
        void setToolOffset(const float offset[3]);
        float get_feed_rate() const;
        void  push_state();
        void  pop_state();
        void check_max_actuator_speeds();
        float to_millimeters( float value ) const { return this->inch_mode ? value * 25.4F : value; }
        float from_millimeters( float value) const { return this->inch_mode ? value/25.4F : value;  }
        void get_axis_position(float position[]) const { memcpy(position, this->last_milestone, sizeof this->last_milestone); }
        wcs_t get_axis_position() const { return wcs_t(last_milestone[0], last_milestone[1], last_milestone[2]); }
        int print_position(uint8_t subcode, char *buf, size_t bufsize) const;
        uint8_t get_current_wcs() const { return current_wcs; }
        std::vector<wcs_t> get_wcs_state() const;
        std::tuple<float, float, float, uint8_t> get_last_probe_position() const { return last_probe_position; }
        void set_last_probe_position(std::tuple<float, float, float, uint8_t> p) { last_probe_position = p; }

        BaseSolution* arm_solution;                           // Selected Arm solution ( millimeters to step calculation )

        // gets accessed by Panel, Endstops, ZProbe
        std::array<StepperMotor*, k_max_actuators> actuators;

        // set by a leveling strategy to transform the target of a move according to the current plan
        std::function<void(float[3])> compensationTransform;

        // Workspace coordinate systems
        wcs_t mcs2wcs(const wcs_t &pos) const;
        wcs_t mcs2wcs(const float *pos) const { return mcs2wcs(wcs_t(pos[0], pos[1], pos[2])); }

        struct {
            bool inch_mode:1;                                 // true for inch mode, false for millimeter mode ( default )
            bool absolute_mode:1;                             // true for absolute mode ( default ), false for relative mode
            bool next_command_is_MCS:1;                       // set by G53
            bool disable_segmentation:1;                      // set to disable segmentation
            uint8_t plane_axis_0:2;                           // Current plane ( XY, XZ, YZ )
            uint8_t plane_axis_1:2;
            uint8_t plane_axis_2:2;
        };

    private:
        void load_config();
        void distance_in_gcode_is_known(Gcode* gcode);
        bool append_milestone( Gcode *gcode, const float target[], float rate_mm_s);
        bool append_line( Gcode* gcode, const float target[], float rate_mm_s);
        bool append_arc( Gcode* gcode, const float target[], const float offset[], float radius, bool is_clockwise );
        bool compute_arc(Gcode* gcode, const float offset[], const float target[]);
        void process_move(Gcode *gcode);

        float theta(float x, float y);
        void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);
        void clearToolOffset();


        std::array<wcs_t, MAX_WCS> wcs_offsets; // these are persistent once saved with M500
        uint8_t current_wcs{0}; // 0 means G54 is enabled this is persistent once saved with M500
        wcs_t g92_offset;
        wcs_t tool_offset; // used for multiple extruders, sets the tool offset for the current extruder applied first
        std::tuple<float, float, float, uint8_t> last_probe_position{0,0,0,0};

        using saved_state_t= std::tuple<float, float, bool, bool, uint8_t>; // save current feedrate and absolute mode, inch mode, current_wcs
        std::stack<saved_state_t> state_stack;               // saves state from M120

        float last_milestone[3]; // Last requested position, in millimeters, which is what we were requested to move to in the gcode after offsets applied but before compensation transform
        float last_machine_position[3]; // Last machine position, which is the position before converting to actuator coordinates (includes compensation transform)
        int8_t motion_mode;                                  // Motion mode for the current received Gcode
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

        // Used by Stepper, Planner
        friend class Planner;
        friend class Stepper;
};


#endif
