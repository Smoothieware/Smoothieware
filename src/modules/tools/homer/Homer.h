/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _HOMER_H
#define _HOMER_H

#include "libs/Module.h"
#include "Actuator.h"

#include <string>
#include <vector>

#define HOME_TO_MAX 2
#define HOME_TO_MIN 1
#define NO_HOME     0

class Gcode;

using std::string;
using std::vector;

enum homing_states {
    HOMING_STATE_FAST_HOME,
    HOMING_STATE_FAST_DEASSERT_ENDSTOP,
    HOMING_STATE_FAST_RETRACT,

    HOMING_STATE_SLOW_HOME,
    HOMING_STATE_SLOW_DEASSERT_ENDSTOP,
    HOMING_STATE_SLOW_RETRACT,

    HOMING_STATE_POST_HOME_MOVE,

    HOMING_STATE_DONE
};

class Homer : public Module
{
public:
    Homer();
    void on_module_loaded();
    void on_gcode_received(void*);
    void on_config_reload( void*);

private:
    struct homing_info
    {
        Actuator* actuator;

        struct {
            bool home_to_max :1;
            bool is_axis     :1;

            enum homing_states state :6;
        };
    };

    string assemble_set_from_gcode(Gcode*);
    void confirm_set(string);
    void home_set(vector<struct homing_info>& actuators);

    string homing_order;
};

#endif /* _HOMER_H */
