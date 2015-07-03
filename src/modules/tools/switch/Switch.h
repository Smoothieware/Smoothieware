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

namespace mbed {
    class PwmOut;
}

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
        enum OUTPUT_TYPE {NONE, SIGMADELTA, DIGITAL, HWPWM};

    private:
        void flip();
        void send_gcode(string msg, StreamOutput* stream);
        bool match_input_on_gcode(const Gcode* gcode) const;
        bool match_input_off_gcode(const Gcode* gcode) const;

        Pin       input_pin;
        float     switch_value;
        OUTPUT_TYPE output_type;
        union {
            Pin          *digital_pin;
            Pwm          *sigmadelta_pin;
            mbed::PwmOut *pwm_pin;
        };
        string    output_on_command;
        string    output_off_command;
        uint16_t  name_checksum;
        uint16_t  input_pin_behavior;
        uint16_t  input_on_command_code;
        uint16_t  input_off_command_code;
        char      input_on_command_letter;
        char      input_off_command_letter;
        struct {
            bool      switch_changed:1;
            bool      input_pin_state:1;
            bool      switch_state;
        };
};

#endif // SWITCH_H
