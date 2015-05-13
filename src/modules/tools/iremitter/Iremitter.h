/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IREMITTER_MODULE_H
#define IREMITTER_MODULE_H

#include "mbed.h"

namespace mbed {
    class PwmOut;
}

class Gcode;

class Iremitter : public Module
{
    public:
        Iremitter();
        virtual ~Iremitter() {};
        void on_module_loaded();

    private:
        void on_config_reload(void *argument);
        void on_gcode_received(void *argument);
        void on_gcode_execute(void *argument);
        
        vector<int> split_raw_code(const char *str, char c = ',');
        
        bool is_executable(Gcode *gcode);
        void trigger_loop();
        void trigger();

        mbed::PwmOut *led_pin;        // PWM pin
        bool led_pin_high;            // LED pin status

        unsigned int m_code;          // M code to trigger the IR LED
        unsigned int frequency;       // PWM LED frequency

        vector<int> raw_code;         // integer raw code vector
        unsigned int raw_code_ptr;    // loop pointer into the raw code vector

        Timeout timeout;              // timeout to delay sending
};

#endif

