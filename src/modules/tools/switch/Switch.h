/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SWITCH_H
#define SWITCH_H

#include "libs/Pin.h"
#include <math.h>


class Switch : public Module {
    public:
        Switch();
        Switch(uint16_t name);

        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);
        void on_gcode_execute(void* argument);
        void on_main_loop(void* argument);
        uint32_t pinpoll_tick(uint32_t dummy);

    private:
        void flip();
        void send_gcode(string msg, StreamOutput* stream);
        bool match_output_gcode(const Gcode* gcode) const;
        bool match_input_gcode(const Gcode* gcode) const;

        uint16_t name_checksum;
        Pin      input_pin;
        uint16_t input_pin_behavior;
        bool     input_pin_state;
        char     input_on_command_letter;
        char     input_off_command_letter;
        uint8_t  input_on_command_code;
        uint8_t  input_off_command_code;
        bool     switch_state;
        float    switch_value;
        bool     switch_changed;
        Pwm      output_pin;
        string   output_on_command;
        string   output_off_command;
};

#endif // SWITCH_H
