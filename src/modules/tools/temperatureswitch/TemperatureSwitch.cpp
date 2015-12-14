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
#include "mri.h"

#define temperatureswitch_checksum                    CHECKSUM("temperatureswitch")
#define enable_checksum                               CHECKSUM("enable")
#define temperatureswitch_hotend_checksum             CHECKSUM("hotend")
#define temperatureswitch_threshold_temp_checksum     CHECKSUM("threshold_temp")
#define temperatureswitch_type_checksum               CHECKSUM("type")
#define temperatureswitch_switch_checksum             CHECKSUM("switch")
#define temperatureswitch_heatup_poll_checksum        CHECKSUM("heatup_poll")
#define temperatureswitch_cooldown_poll_checksum      CHECKSUM("cooldown_poll")
#define temperatureswitch_trigger_checksum            CHECKSUM("trigger")
#define temperatureswitch_inverted_checksum           CHECKSUM("inverted")
#define temperatureswitch_arm_command_checksum        CHECKSUM("arm_mcode")
#define designator_checksum                           CHECKSUM("designator")

TemperatureSwitch::TemperatureSwitch()
{
}

TemperatureSwitch::~TemperatureSwitch()
{
    THEKERNEL->unregister_for_event(ON_SECOND_TICK, this);
    THEKERNEL->unregister_for_event(ON_GCODE_RECEIVED, this);
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

TemperatureSwitch* TemperatureSwitch::load_config(uint16_t modcs)
{
    // see if enabled
    if (!THEKERNEL->config->value(temperatureswitch_checksum, modcs, enable_checksum)->by_default(false)->as_bool()) {
        return nullptr;
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

    if(designator == 0) return nullptr; // no designator then not valid

    // load settings from config file
    string switchname = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_switch_checksum)->by_default("")->as_string();
    if(switchname.empty()) {
        // handle old configs where this was called type @DEPRECATED
        switchname = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_type_checksum)->by_default("")->as_string();
        if(switchname.empty()) {
            // no switch specified so invalid entry
            THEKERNEL->streams->printf("WARNING TEMPERATURESWITCH: no switch specified\n");
            return nullptr;
        }
    }

    // create a new temperature switch module
    TemperatureSwitch *ts= new TemperatureSwitch();

    // save designator
    ts->designator= designator;

    // if we should turn the switch on or off when trigger is hit
    ts->inverted = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_inverted_checksum)->by_default(false)->as_bool();

    // if we should trigger when above and below, or when rising through, or when falling through the specified temp
    string trig = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_trigger_checksum)->by_default("level")->as_string();
    if(trig == "level") ts->trigger= LEVEL;
    else if(trig == "rising") ts->trigger= RISING;
    else if(trig == "falling") ts->trigger= FALLING;
    else ts->trigger= LEVEL;

    // the mcode used to arm the switch
    ts->arm_mcode = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_arm_command_checksum)->by_default(0)->as_number();

    ts->temperatureswitch_switch_cs= get_checksum(switchname); // checksum of the switch to use

    ts->temperatureswitch_threshold_temp = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_threshold_temp_checksum)->by_default(50.0f)->as_number();

    // these are to tune the heatup and cooldown polling frequencies
    ts->temperatureswitch_heatup_poll = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_heatup_poll_checksum)->by_default(15)->as_number();
    ts->temperatureswitch_cooldown_poll = THEKERNEL->config->value(temperatureswitch_checksum, modcs, temperatureswitch_cooldown_poll_checksum)->by_default(60)->as_number();
    ts->current_delay = ts->temperatureswitch_heatup_poll;

    // set initial state
    ts->current_state= NONE;
    ts->second_counter = ts->current_delay; // do test immediately on first second_tick
    // if not defined then always armed, otherwise start out disarmed
    ts->armed= (ts->arm_mcode == 0);

    // Register for events
    ts->register_for_event(ON_SECOND_TICK);

    if(ts->arm_mcode != 0) {
        ts->register_for_event(ON_GCODE_RECEIVED);
    }
    return ts;
}

void TemperatureSwitch::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if(gcode->has_m && gcode->m == this->arm_mcode) {
        this->armed= (gcode->has_letter('S') && gcode->get_value('S') != 0);
        gcode->stream->printf("temperature switch %s\n", this->armed ? "armed" : "disarmed");
    }
}

// Called once a second but we only need to service on the cooldown and heatup poll intervals
void TemperatureSwitch::on_second_tick(void *argument)
{
    second_counter++;
    if (second_counter < current_delay) return;

    second_counter = 0;
    float current_temp = this->get_highest_temperature();

    if (current_temp >= this->temperatureswitch_threshold_temp) {
        set_state(HIGH_TEMP);

    } else {
        set_state(LOW_TEMP);
   }
}

void TemperatureSwitch::set_state(STATE state)
{
    if(state == this->current_state) return; // state did not change

    // state has changed
    switch(this->trigger) {
        case LEVEL:
            // switch on or off depending on HIGH or LOW
            set_switch(state == HIGH_TEMP);
            break;

        case RISING:
            // switch on if rising edge
            if(this->current_state == LOW_TEMP && state == HIGH_TEMP) set_switch(true);
            break;

        case FALLING:
            // switch off if falling edge
            if(this->current_state == HIGH_TEMP && state == LOW_TEMP) set_switch(false);
            break;
    }

    this->current_delay = state == HIGH_TEMP ? this->temperatureswitch_cooldown_poll : this->temperatureswitch_heatup_poll;
    this->current_state= state;
}

// Get the highest temperature from the set of temperature controllers
float TemperatureSwitch::get_highest_temperature()
{
    float high_temp = 0.0;

    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok) {
        for (auto &c : controllers) {
            // check if this controller's temp is the highest and save it if so
            if (c.designator[0] == this->designator && c.current_temperature > high_temp) {
                high_temp = c.current_temperature;
            }
        }
    }

    return high_temp;
}

// Turn the switch on (true) or off (false)
void TemperatureSwitch::set_switch(bool switch_state)
{
    if(!this->armed) return; // do not actually switch anything if not armed

    if(this->arm_mcode != 0 && this->trigger != LEVEL) {
        // if edge triggered we only trigger once per arming, if level triggered we switch as long as we are armed
        this->armed= false;
    }

    if(this->inverted) switch_state= !switch_state; // turn switch on or off inverted

    // get current switch state
    struct pad_switch pad;
    bool ok = PublicData::get_value(switch_checksum, this->temperatureswitch_switch_cs, 0, &pad);
    if (!ok) {
        THEKERNEL->streams->printf("// Failed to get switch state.\r\n");
        return;
    }

    if(pad.state == switch_state) return; // switch is already in the requested state

    ok = PublicData::set_value(switch_checksum, this->temperatureswitch_switch_cs, state_checksum, &switch_state);
    if (!ok) {
        THEKERNEL->streams->printf("// Failed changing switch state.\r\n");
    }
}
