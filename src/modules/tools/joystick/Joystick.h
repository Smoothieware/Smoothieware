/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include "libs/Pin.h"
#include "RingBuffer.h"

#define QUEUE_LEN 32

class Joystick : public Module {
    public:
        Joystick();
        ~Joystick();

        void on_module_loaded();
        void on_config_reload(void* argument);
        uint32_t update_tick(uint32_t);
        void on_gcode_received(void* argument);

        int* read_pos();
        float* get_normalized(int* pos);
        float get_speed(float pos);

        void acceleration_tick(void);
        void accelerate(int c);

private:
        Pin axis_pin[2];
        Pin endstop_pin[6];
        float max_speed;
        float dead_zone;
        int zero_offset [2] = {};
        float nonlinearity;
        int averaging_length;

        float* position;
        bool direction[2];
        volatile float target_feedrate[2];
        volatile bool enabled[2];

        RingBuffer<uint16_t,QUEUE_LEN> queue[2];  // Queue of readings
};

#endif //JOYSTICK_H