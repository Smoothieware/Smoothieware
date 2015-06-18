/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SWITCH_H
#define SWITCH_H

#include "Pin.h"
#include "Pwm.h"
#include <math.h>

#include <string>
using std::string;

class Gcode;
class StreamOutput;

class Switch : public Module {
    public:
        Switch();
        Switch(uint16_t name);

        void on_module_loaded();
        void on_main_loop(void *argument);
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        uint32_t pinpoll_tick(uint32_t dummy);
        uint32_t pinpulse_tick(uint32_t dummy);

    private:
        void flip();
        void send_gcode(string msg, StreamOutput* stream);
        bool match_input_on_gcode(const Gcode* gcode) const;
        bool match_input_off_gcode(const Gcode* gcode) const;
        
        uint16_t  name_checksum;
        Pin       input_pin;
        Pwm       output_pin;
        
        float     switch_value;
        
        string    output_on_command;
        string    output_off_command;
        
        uint16_t  input_on_command_code;
        uint16_t  input_off_command_code;
        char      input_on_command_letter;
        char      input_off_command_letter;
        struct {
            bool      switch_changed:1;
            bool      input_pin_state:1;
            bool      switch_state:1;
            bool      is_input_momentary:1;
            bool      is_output_pwm:1;
            bool      is_output_pulse:1;
        };
};

#endif // SWITCH_H
