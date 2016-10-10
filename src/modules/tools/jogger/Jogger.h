
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

class Jogger : public Module {
public:
    Jogger();

    void on_module_loaded();
    void on_config_reload(void* argument);
    void on_gcode_received(void* argument);
    uint32_t update_tick(uint32_t);

private:
    float get_speed(float pos);
    //void acceleration_tick(void);
    //void accelerate(int c);

    float max_speed = 10.0f; //maximum allowed speed (defaults to slow 10 mm/min)
    float dead_zone = 0.002f; //distance from 0 in which small joystick movement is ignored
    float nonlinearity = 1.0f; //nonlinearity parameter for scaling of joystick values

    float position[NUM_JOG_AXES] = {}; //keeps track of the joystick positions for each axis
    bool direction[NUM_JOG_AXES]; //keeps track of the direction of the steppers (true = positive)
    float target_speed[NUM_JOG_AXES] = {}; //volatile? keeps track of the target speed of each axis
    bool enabled[NUM_JOG_AXES]; //volatile? keeps track of which steppers are enabled

    int refresh_rate = 100; //number of jog speed updates per second (absolute max = base_step_frequency (100 kHz))
    float step_scale_factor = 0.1f; //max number of mm per requested step

    uint16_t axis_data_source[NUM_JOG_AXES] = {}; //checksums of the joystick module name from which to obtain data for axis n

};

#endif //JOGGER_H
