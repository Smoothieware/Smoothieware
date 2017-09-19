/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ANALOG_SPINDLE_MODULE_H
#define ANALOG_SPINDLE_MODULE_H

#include "SpindleControl.h"

namespace mbed {
    class PwmOut;
}

class Pin;

// This module implements control of the spindle speed by seting a PWM from 0-100% which needs
// to be converted to 0-10V by an external circuit
class AnalogSpindleControl: public SpindleControl {
    public:
        AnalogSpindleControl() {};
        virtual ~AnalogSpindleControl() {};
        void on_module_loaded();
        
    private:
       
        Pin *switch_on; // digital output for switching the VFD on 
        mbed::PwmOut *pwm_pin; // PWM output for spindle speed control
        bool output_inverted;
        
        int target_rpm;
        int min_rpm;
        int max_rpm;

        void turn_on(void);
        void turn_off(void);
        void set_speed(int);
        void report_speed(void);
        void update_pwm(float); 
};

#endif

