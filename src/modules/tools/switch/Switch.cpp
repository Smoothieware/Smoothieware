/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "Switch.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "SwitchPublicAccess.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"

#include "MRI_Hooks.h"

#define    startup_state_checksum       CHECKSUM("startup_state")
#define    startup_value_checksum       CHECKSUM("startup_value")
#define    input_pin_checksum           CHECKSUM("input_pin")
#define    input_pin_behavior_checksum  CHECKSUM("input_pin_behavior")
#define    toggle_checksum              CHECKSUM("toggle")
#define    momentary_checksum           CHECKSUM("momentary")
#define    input_on_command_checksum    CHECKSUM("input_on_command")
#define    input_off_command_checksum   CHECKSUM("input_off_command")
#define    output_pin_checksum          CHECKSUM("output_pin")
#define    output_type_checksum         CHECKSUM("output_type")
#define    max_pwm_checksum             CHECKSUM("max_pwm")
#define    output_on_command_checksum   CHECKSUM("output_on_command")
#define    output_off_command_checksum  CHECKSUM("output_off_command")

Switch::Switch() {}

Switch::Switch(uint16_t name)
{
    this->name_checksum = name;
    //this->dummy_stream = &(StreamOutput::NullStream);
}

void Switch::on_module_loaded()
{
    this->switch_changed = false;

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Settings
    this->on_config_reload(this);
}


// Get config
void Switch::on_config_reload(void *argument)
{
    this->input_pin.from_string( THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->input_pin_behavior =   THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_behavior_checksum )->by_default(momentary_checksum)->as_number();
    std::string input_on_command =    THEKERNEL->config->value(switch_checksum, this->name_checksum, input_on_command_checksum )->by_default("")->as_string();
    std::string input_off_command =   THEKERNEL->config->value(switch_checksum, this->name_checksum, input_off_command_checksum )->by_default("")->as_string();
    this->output_pin.from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
    this->output_on_command =    THEKERNEL->config->value(switch_checksum, this->name_checksum, output_on_command_checksum )->by_default("")->as_string();
    this->output_off_command =   THEKERNEL->config->value(switch_checksum, this->name_checksum, output_off_command_checksum )->by_default("")->as_string();
    this->switch_state =         THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_state_checksum )->by_default(false)->as_bool();
    string type =                THEKERNEL->config->value(switch_checksum, this->name_checksum, output_type_checksum )->by_default("pwm")->as_string();

    if(type == "pwm") this->output_type= PWM;
    else if(type == "digital") this->output_type= DIGITAL;
    else this->output_type= PWM; // unkown type default to pwm

    if(this->output_pin.connected()) {
        if(this->output_type == PWM) {
            this->output_pin.max_pwm(THEKERNEL->config->value(switch_checksum, this->name_checksum, max_pwm_checksum )->by_default(255)->as_number());
            this->switch_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_value_checksum )->by_default(this->output_pin.max_pwm())->as_number();
            if(this->switch_state) {
                this->output_pin.pwm(this->switch_value); // will be truncated to max_pwm
            } else {
                this->output_pin.set(false);
            }
        } else {
            this->output_pin.set(this->switch_state);
        }
    }

    set_low_on_debug(output_pin.port_number, output_pin.pin);

    // Set the on/off command codes, Use GCode to do the parsing
    input_on_command_letter = 0;
    input_off_command_letter = 0;

    if(!input_on_command.empty()) {
        Gcode gc(input_on_command, NULL);
        if(gc.has_g) {
            input_on_command_letter = 'G';
            input_on_command_code = gc.g;
        } else if(gc.has_m) {
            input_on_command_letter = 'M';
            input_on_command_code = gc.m;
        }
    }
    if(!input_off_command.empty()) {
        Gcode gc(input_off_command, NULL);
        if(gc.has_g) {
            input_off_command_letter = 'G';
            input_off_command_code = gc.g;
        } else if(gc.has_m) {
            input_off_command_letter = 'M';
            input_off_command_code = gc.m;
        }
    }

    if(input_pin.connected()) {
        // set to initial state
        this->input_pin_state = this->input_pin.get();
        // input pin polling
        THEKERNEL->slow_ticker->attach( 100, this, &Switch::pinpoll_tick);
    }

    if(this->output_type == PWM && this->output_pin.connected()) {
        // PWM
        THEKERNEL->slow_ticker->attach(1000, &this->output_pin, &Pwm::on_tick);
    }
}

bool Switch::match_input_on_gcode(const Gcode *gcode) const
{
    return ((input_on_command_letter == 'M' && gcode->has_m && gcode->m == input_on_command_code) ||
            (input_on_command_letter == 'G' && gcode->has_g && gcode->g == input_on_command_code));
}

bool Switch::match_input_off_gcode(const Gcode *gcode) const
{
    return ((input_off_command_letter == 'M' && gcode->has_m && gcode->m == input_off_command_code) ||
            (input_off_command_letter == 'G' && gcode->has_g && gcode->g == input_off_command_code));
}

void Switch::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Add the gcode to the queue ourselves if we need it
    if (match_input_on_gcode(gcode) || match_input_off_gcode(gcode)) {
        THEKERNEL->conveyor->append_gcode(gcode);
    }
}

// Turn pin on and off
void Switch::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if(match_input_on_gcode(gcode)) {
        int v;
        this->switch_state = true;
        if (this->output_type == PWM) {
            // PWM output pin turn on
            if(gcode->has_letter('S')) {
                v = round(gcode->get_value('S') * output_pin.max_pwm() / 255.0); // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
                this->output_pin.pwm(v);
            } else {
                this->output_pin.pwm(this->switch_value);
            }
        } else {
            // logic pin turn on
            this->output_pin.set(true);
        }
    } else if(match_input_off_gcode(gcode)) {
        this->switch_state = false;
        if (this->output_type == PWM) {
            // PWM output pin
            this->output_pin.set(false);
        } else {
            // logic pin turn off
            this->output_pin.set(false);
        }
    }
}

void Switch::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return; // likely fan, but could be anything

    // ok this is targeted at us, so send back the requested data
    // this must be static as it will be accessed long after we have returned
    static struct pad_switch pad;
    pad.name = this->name_checksum;
    pad.state = this->switch_state;
    pad.value = this->switch_value;

    pdr->set_data_ptr(&pad);
    pdr->set_taken();
}

void Switch::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return; // likely fan, but could be anything

    // ok this is targeted at us, so set the value
    if(pdr->third_element_is(state_checksum)) {
        bool t = *static_cast<bool *>(pdr->get_data_ptr());
        this->switch_state = t;
        pdr->set_taken();
        this->switch_changed= true;

    } else if(pdr->third_element_is(value_checksum)) {
        float t = *static_cast<float *>(pdr->get_data_ptr());
        this->switch_value = t;
        this->switch_changed= true;
        pdr->set_taken();
    }
}

void Switch::on_main_loop(void *argument)
{
    if(this->switch_changed) {
        if(this->switch_state) {
            if(!this->output_on_command.empty()) this->send_gcode( this->output_on_command, &(StreamOutput::NullStream) );
            if(this->output_pin.connected()) {
                if(this->output_type == PWM)
                    this->output_pin.pwm(this->switch_value); // this requires the value has been set otherwise it swicthes on to whatever it last was
                else
                    this->output_pin.set(true);
            }

        } else {
            if(!this->output_off_command.empty()) this->send_gcode( this->output_off_command, &(StreamOutput::NullStream) );
            if(this->output_pin.connected()) {
                if(this->output_type == PWM)
                    this->output_pin.set(false);
                else
                    this->output_pin.set(false);
            }
        }
        this->switch_changed = false;
    }
}

// TODO Make this use InterruptIn
// Check the state of the button and act accordingly
uint32_t Switch::pinpoll_tick(uint32_t dummy)
{
    if(!input_pin.connected()) return 0;

    // If pin changed
    bool current_state = this->input_pin.get();
    if(this->input_pin_state != current_state) {
        this->input_pin_state = current_state;
        // If pin high
        if( this->input_pin_state ) {
            // if switch is a toggle switch
            if( this->input_pin_behavior == toggle_checksum ) {
                this->flip();
                // else default is momentary
            } else {
                this->flip();
            }
            // else if button released
        } else {
            // if switch is momentary
            if( this->input_pin_behavior == momentary_checksum ) {
                this->flip();
            }
        }
    }
    return 0;
}

void Switch::flip()
{
    this->switch_state = !this->switch_state;
    this->switch_changed = true;
}

void Switch::send_gcode(std::string msg, StreamOutput *stream)
{
    struct SerialMessage message;
    message.message = msg;
    message.stream = stream;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}

