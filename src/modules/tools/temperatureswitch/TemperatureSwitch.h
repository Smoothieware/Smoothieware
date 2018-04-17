/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/*
TemperatureSwitch is an optional module that will automatically turn on or off a switch
based on a setpoint temperature. It is commonly used to turn on/off a cooling fan or water pump
to cool the hot end's cold zone. Specifically, it turns one of the small MOSFETs on or off.

Author: Michael Hackney, mhackney@eclecticangler.com
*/

#ifndef TEMPERATURESWITCH_MODULE_H
#define TEMPERATURESWITCH_MODULE_H

using namespace std;

#include "libs/Module.h"
#include <string>
#include <vector>

class TemperatureSwitch : public Module
{
    public:
        TemperatureSwitch();
        ~TemperatureSwitch();
        void on_module_loaded();
        void on_second_tick(void *argument);
        void on_gcode_received(void *argument);
        TemperatureSwitch* load_config(uint16_t modcs);

        bool is_armed() const { return armed; }

    private:
        enum TRIGGER_TYPE {LEVEL, RISING, FALLING};
        enum STATE {NONE, HIGH_TEMP, LOW_TEMP};

        // get the highest temperature from the set of configured temperature controllers
        float get_highest_temperature();

        // turn the switch on or off
        void set_switch(bool cooler_state);

        // temperature has changed state
        void set_state(STATE state);

        // temperatureswitch.hotend.threshold_temp
        float temperatureswitch_threshold_temp;

        // temperatureswitch.hotend.switch
        uint16_t temperatureswitch_switch_cs;

        // check temps on heatup every X seconds
        // this can be set in config: temperatureswitch.hotend.heatup_poll
        uint16_t temperatureswitch_heatup_poll;

        // check temps on cooldown every X seconds
        // this can be set in config: temperatureswitch.hotend.cooldown_poll
        uint16_t temperatureswitch_cooldown_poll;

        // our internal second counter
        uint16_t second_counter;

        // we are delaying for this many seconds
        uint16_t current_delay;

        // the mcode that will arm the switch, 0 means always armed
        uint16_t arm_mcode;

        struct {
            char designator:8;
            bool inverted:1;
            bool armed:1;
            TRIGGER_TYPE trigger:2;
            STATE current_state:2;
        };
};

#endif
