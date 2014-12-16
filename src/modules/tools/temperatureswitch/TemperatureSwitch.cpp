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

#include "TemperatureSwitch.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "SwitchPublicAccess.h"

#include "utils.h"
#include "Gcode.h"
#include "Config.h"
#include "ConfigValue.h"
#include "checksumm.h"
#include "PublicData.h"
#include "StreamOutputPool.h"
#include "TemperatureControlPool.h"

#define temperatureswitch_checksum                    CHECKSUM("temperatureswitch")
#define enable_checksum                               CHECKSUM("enable")
#define temperatureswitch_hotend_checksum             CHECKSUM("hotend")
#define temperatureswitch_threshold_temp_checksum     CHECKSUM("threshold_temp")
#define temperatureswitch_type_checksum               CHECKSUM("type")
#define temperatureswitch_switch_checksum             CHECKSUM("switch")
#define temperatureswitch_heatup_poll_checksum        CHECKSUM("heatup_poll")
#define temperatureswitch_cooldown_poll_checksum      CHECKSUM("cooldown_poll")
#define designator_checksum                           CHECKSUM("designator")

TemperatureSwitch::TemperatureSwitch()
{
    this->temperatureswitch_state = false;
    this->second_counter = 0;
}

// Load module
void TemperatureSwitch::on_module_loaded()
{
    vector<uint16_t> modulist;
    // allow for multiple temperature switches
    THEKERNEL->config->get_module_list(&modulist, temperatureswitch_checksum);
    for (auto m : modulist) {
        load_config(m);
    }

    // no longer need this instance as it is just used to load the other instances
    delete this;
}


bool TemperatureSwitch::load_config(uint16_t modcs)
{
    // see if enabled
    if (!THEKERNEL->config->value(temperatureswitch_checksum, modcs, enable_checksum)->by_default(false)->as_bool()) {
        return false;
    }

    // create a temperature control and load settings

    char designator= 0;
    string s= THEKERNEL->config->value(temperatureswitch_checksum, modcs, designator_checksum)->by_default("")->as_string();
    if(s.empty()){
        // for backward compatibility temperatureswitch.hotend will need designator 'T' by default @DEPRECATED
        if(modcs == temperatureswitch_hotend_checksum) designator= 'T';

    }else{
        designator= s[0];
    }

    if(designator == 0) return false; // no designator then not valid

    // create a new temperature switch module
    TemperatureSwitch *ts= new TemperatureSwitch();

    // get the list of temperature controllers and remove any that don't have designator == specified designator
    auto& tempcontrollers= THEKERNEL->temperature_control_pool->get_controllers();

    // see what its designator is and add to list of it the one we specified
    void *returned_temp;
    for (auto controller : tempcontrollers) {
        bool temp_ok = PublicData::get_value(temperature_control_checksum, controller, current_temperature_checksum, &returned_temp);
        if (temp_ok) {
            struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_temp);
            if (temp.designator[0] == designator) {
                ts->temp_controllers.push_back(controller);
            }
        }
    }

    // if we don't have any matching controllers, then not valid
    if (ts->temp_controllers.empty()) {
        delete ts;
        return false;
    }

    // load settings from config file
    s = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_switch_checksum)->by_default("")->as_string();
    if(s.empty()) {
        // handle old configs where this was called type @DEPRECATED
        s = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_type_checksum)->by_default("")->as_string();
        if(s.empty()) {
            // no switch specified so invalid entry
            delete this;
            return false;
        }
    }
    ts->temperatureswitch_switch_cs= get_checksum(s); // checksum of the switch to use

    ts->temperatureswitch_threshold_temp = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_threshold_temp_checksum)->by_default(50.0f)->as_number();

    // these are to tune the heatup and cooldown polling frequencies
    ts->temperatureswitch_heatup_poll = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_heatup_poll_checksum)->by_default(15)->as_number();
    ts->temperatureswitch_cooldown_poll = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_poll_checksum)->by_default(60)->as_number();
    ts->current_delay = ts->temperatureswitch_heatup_poll;

    // Register for events
    ts->register_for_event(ON_SECOND_TICK);

    return true;
}

// Called once a second but we only need to service on the cooldown and heatup poll intervals
void TemperatureSwitch::on_second_tick(void *argument)
{
    second_counter++;
    if (second_counter < current_delay) {
        return;
    } else {
        second_counter = 0;
        float current_temp = this->get_highest_temperature();

        if (current_temp >= this->temperatureswitch_threshold_temp) {
            // temp >= threshold temp, turn the cooler switch on if it isn't already
            if (!temperatureswitch_state) {
                set_switch(true);
                current_delay = temperatureswitch_cooldown_poll;
            }
        } else {
            // temp < threshold temp, turn the cooler switch off if it isn't already
            if (temperatureswitch_state) {
                set_switch(false);
                current_delay = temperatureswitch_heatup_poll;
            }
        }
    }
}

// Get the highest temperature from the set of temperature controllers
float TemperatureSwitch::get_highest_temperature()
{
    void *returned_temp;
    float high_temp = 0.0;

    for (auto controller : temp_controllers) {
        bool temp_ok = PublicData::get_value(temperature_control_checksum, controller, current_temperature_checksum, &returned_temp);
        if (temp_ok) {
            struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_temp);
            // check if this controller's temp is the highest and save it if so
            if (temp.current_temperature > high_temp) {
                high_temp = temp.current_temperature;
            }
        }
    }
    return high_temp;
}

// Turn the switch on (true) or off (false)
void TemperatureSwitch::set_switch(bool switch_state)
{
    this->temperatureswitch_state = switch_state;
    bool ok = PublicData::set_value(switch_checksum, this->temperatureswitch_switch_cs, state_checksum, &this->temperatureswitch_state);
    if (!ok) {
        THEKERNEL->streams->printf("Failed changing switch state.\r\n");
    }
}
