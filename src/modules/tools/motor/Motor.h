/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "Pin.h"
#include "utils.h"
#include <stdint.h>

#define HIGH 1
#define LOW  0

class Pin;

class Motor : public Module{
    public:
        Motor();
        virtual ~Motor() {};
        void on_module_loaded();
        void on_gcode_received(void *argument);
        uint32_t tick(uint32_t dummy);

        enum STATUS {NONE, HOMING, MOVING_TO, MOVING_FOREVER};

    private:
        Pin tick_pin;				      // Pin for rotation tick
        Pin home_pin;				      // Pin for homing
        Pin clockwise_pin;        // Pin for moving clockwise
        Pin counter_clockwise_pin;// Pin for moving counter clockwise

        bool current_home_pin_value; // Current value of the home pin, so we can check if it changes

        STATUS status;

};
