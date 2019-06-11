/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "Pin.h"
#include "Pwm.h"
#include "SoftPWM.h"

#include <math.h>
#include <string>

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
        void on_halt(void *arg);
        void on_suspend(void *arg);

        uint32_t pinpoll_tick(uint32_t dummy);
        enum OUTPUT_TYPE {NONE, SIGMADELTA, DIGITAL, HWPWM, SWPWM};

    private:
        void flip();
        void send_gcode(std::string msg, StreamOutput* stream);
        bool match_input_on_gcode(const Gcode* gcode) const;
        bool match_input_off_gcode(const Gcode* gcode) const;

        float switch_value;
        float default_on_value;

        OUTPUT_TYPE output_type;
        union {
            Pin          *input_pin;
            Pin          *digital_pin;
            Pwm          *sigmadelta_pin;
            mbed::PwmOut *pwm_pin;
            SoftPWM      *swpwm_pin;
        };
        std::string    output_on_command;
        std::string    output_off_command;
        struct {
            uint16_t  name_checksum:16;
            uint16_t  input_pin_behavior:16;
            uint16_t  input_on_command_code:16;
            uint16_t  input_off_command_code:16;
            char      input_on_command_letter:8;
            char      input_off_command_letter:8;
            uint8_t   subcode:4;
            bool      switch_changed:1;
            bool      input_pin_state:1;
            bool      switch_state:1;
            bool      ignore_on_halt:1;
            bool      failsafe:1;
        };
};
