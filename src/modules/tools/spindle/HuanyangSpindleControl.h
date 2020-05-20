/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HUANYANG_SPINDLE_CONTROL_MODULE_H
#define HUANYANG_SPINDLE_CONTROL_MODULE_H

#include "ModbusSpindleControl.h"
#include <stdint.h>

// This module implements Modbus control for spindle control over Modbus.
class HuanyangSpindleControl: public ModbusSpindleControl {
    public:
        HuanyangSpindleControl() {};
        virtual ~HuanyangSpindleControl() {};
        
    private:
        
        void turn_on(void);
        void turn_off(void);
        void set_speed(int);
        void report_speed(void);
};

#endif

