
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
    void acceleration_tick(void);
    void accelerate(int c);

    float max_speed = 10.0f;
    float dead_zone = 0.002f;
    float nonlinearity = 1.0f;

    float position[NUM_JOG_AXES] = {}; //keeps track of the joystick positions for each axis
    bool direction[NUM_JOG_AXES]; //keeps track of the direction of the steppers (true = positive)
    float target_speed[NUM_JOG_AXES] = {}; //volatile? keeps track of the target speed of each axis
    bool enabled[NUM_JOG_AXES]; //volatile? keeps track of which steppers are enabled

    int refresh_interval = 1000; //number of milliseconds between jog speed updates (max 1000)

    uint16_t axis_data_source[NUM_JOG_AXES] = {}; //checksums of the joystick module name from which to obtain data for axis n

};

#endif //JOGGER_H
