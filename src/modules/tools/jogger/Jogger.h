
/*
this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef JOGGER_H
#define JOGGER_H

#define NUM_JOG_AXES 3

#include "Module.h"
#include <stdint.h>
#include <string>
#include <vector>

class Jogger : public Module {
public:
    Jogger();

    void on_module_loaded();
    void on_config_reload(void* argument);
    void on_gcode_received(void* argument);
    uint32_t update_tick(uint32_t dummy);
    void on_main_loop(void* argument);

private:
    float get_speed(float pos); //function to return a speed given a unitless joystick reading

    float max_speed{ 10.0f }; //maximum allowed speed (defaults to slow 10 mm/min)
    float dead_zone{ 0.002f }; //distance from 0 in which small joystick movement is ignored
    float nonlinearity{ 1.0f }; //nonlinearity parameter for scaling of joystick values

    float position[NUM_JOG_AXES]; //keeps track of the joystick positions for each axis
    float target_speed[NUM_JOG_AXES]; //keeps track of the target speed of each axis
    char axis_letter[NUM_JOG_AXES]; //keeps track of which machine axis is controlled by each joystick axis
    std::vector<std::string> jog_axes; //stores the list of machine axes controlled by the joystick
    unsigned int axis_index{ 0 }; //stores which item in the list of machine axis configs is in use
    struct {
        bool is_active:1; //keeps track of whether the joystick is being moved
        bool is_jogging:1; //keeps track of whether the robot is jogging
        bool do_reading:1; //flag for when a joystick reading should occur
    };

    int refresh_rate{ 100 }; //number of jog speed updates per second (absolute max = base_step_frequency (100 kHz))
    int segment_frequency{ 100 }; //approximate number of segments per second during jogging

    uint16_t m_code_set{ 777 }; //m-code number to use when setting jog axes
    uint16_t m_code_toggle{ 778 }; //m-code number to use when toggling jog axes

    uint16_t axis_data_source[NUM_JOG_AXES]; //checksums of the joystick module name from which to obtain data for axis n

    void update_Joystick(void);
    void update_Jogging(void);
    std::string get_Gcode(void);
    void update_Axes(std::string axisstr);
};

#endif //JOGGER_H
