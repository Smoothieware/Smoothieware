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

#define temperatureswitch_checksum                    CHECKSUM("temperatureswitch")
#define temperatureswitch_enable_checksum             CHECKSUM("enable")
#define temperatureswitch_hotend_checksum             CHECKSUM("hotend")
#define temperatureswitch_threshold_temp_checksum     CHECKSUM("threshold_temp")
#define temperatureswitch_type_checksum               CHECKSUM("type")
#define temperatureswitch_heatup_poll_checksum        CHECKSUM("heatup_poll")
#define temperatureswitch_cooldown_poll_checksum      CHECKSUM("cooldown_poll")

TemperatureSwitch::TemperatureSwitch()
{
}

// Load module
void TemperatureSwitch::on_module_loaded()
{
    // free up space if not loaded
    if (!THEKERNEL->config->value(temperatureswitch_checksum, temperatureswitch_hotend_checksum, temperatureswitch_enable_checksum)->by_default(false)->as_bool()) {
        delete this;
        return;
    }

    // settings
    this->on_config_reload(this);
}

// Get config
void TemperatureSwitch::on_config_reload(void *argument)
{
    // get the list of temperature controllers and remove any that fon't have designator == "T"
    vector<uint16_t> controller_list;  
    THEKERNEL->config->get_module_list(&controller_list, temperature_control_checksum);
    
    void *returned_temp;
    for (auto controller : controller_list) {
        bool temp_ok = PublicData::get_value(temperature_control_checksum, controller, current_temperature_checksum, &returned_temp);
        if (temp_ok) {
            struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_temp);
            // if the controller is a hotend (designator == "T") then keep it
            if (temp.designator.substr(0, 1) == "T") {
                temp_controllers.push_back(controller);
            }
        }
    }
    
    // if we don't have any controllers, free up space
    if (temp_controllers.empty()) {
        delete this;
        return;
    }   
    
    // load settings from config file
    this->temperatureswitch_state = false;
    this->temperatureswitch_type = THEKERNEL->config->value(temperatureswitch_checksum, temperatureswitch_hotend_checksum, temperatureswitch_type_checksum)->by_default("")->as_string();
    this->temperatureswitch_threshold_temp = THEKERNEL->config->value(temperatureswitch_checksum, temperatureswitch_hotend_checksum, temperatureswitch_threshold_temp_checksum)->by_default(50.0f)->as_number();    

    // these are to tune the heatup and cooldown polling frequencies
    this->temperatureswitch_heatup_poll = THEKERNEL->config->value(temperatureswitch_checksum, temperatureswitch_hotend_checksum, temperatureswitch_heatup_poll_checksum)->by_default(15)->as_number();   
    this->temperatureswitch_cooldown_poll = THEKERNEL->config->value(temperatureswitch_checksum, temperatureswitch_hotend_checksum, temperatureswitch_cooldown_poll_checksum)->by_default(60)->as_number();

    second_counter = 0;
    current_delay = this->temperatureswitch_heatup_poll;

    // Register for events
    this->register_for_event(ON_SECOND_TICK);
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
            if (temp.current_temperature > high_temp && temp.designator.substr(0, 1) == "T") {
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
    bool ok = PublicData::set_value(switch_checksum, get_checksum(this->temperatureswitch_type), state_checksum, &this->temperatureswitch_state);
    if (!ok) {
        THEKERNEL->streams->printf("Failed changing switch state.\r\n");
    }
}
