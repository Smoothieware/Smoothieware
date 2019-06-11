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
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#include "PwmOut.h"

#include "MRI_Hooks.h"

#include <algorithm>

#define    startup_state_checksum       CHECKSUM("startup_state")
#define    startup_value_checksum       CHECKSUM("startup_value")
#define    default_on_value_checksum    CHECKSUM("default_on_value")
#define    input_pin_checksum           CHECKSUM("input_pin")
#define    input_pin_behavior_checksum  CHECKSUM("input_pin_behavior")
#define    toggle_checksum              CHECKSUM("toggle")
#define    momentary_checksum           CHECKSUM("momentary")
#define    command_subcode_checksum     CHECKSUM("subcode")
#define    input_on_command_checksum    CHECKSUM("input_on_command")
#define    input_off_command_checksum   CHECKSUM("input_off_command")
#define    output_pin_checksum          CHECKSUM("output_pin")
#define    output_type_checksum         CHECKSUM("output_type")
#define    max_pwm_checksum             CHECKSUM("max_pwm")
#define    output_on_command_checksum   CHECKSUM("output_on_command")
#define    output_off_command_checksum  CHECKSUM("output_off_command")
#define    pwm_period_ms_checksum       CHECKSUM("pwm_period_ms")
#define    failsafe_checksum            CHECKSUM("failsafe_set_to")
#define    ignore_onhalt_checksum       CHECKSUM("ignore_on_halt")

#define ROUND2DP(x) (roundf(x * 1e2F) / 1e2F)

Switch::Switch() {}

Switch::Switch(uint16_t name)
{
    this->name_checksum = name;
    //this->dummy_stream = &(StreamOutput::NullStream);
}

// set the pin to the fail safe value on halt
void Switch::on_halt(void *arg)
{
    if(arg == nullptr) {
        if(this->ignore_on_halt) return;

        // set pin to failsafe value
        switch(this->output_type) {
            case DIGITAL: this->digital_pin->set(this->failsafe); break;
            case SIGMADELTA: this->sigmadelta_pin->set(this->failsafe); break;
            case HWPWM: this->pwm_pin->write(switch_value/100.0F); break;
            case SWPWM: this->swpwm_pin->write(switch_value/100.0F); break;
            case NONE: return;
        }
        this->switch_state= this->failsafe;
    }
}
// set the pin to the fail safe value on halt
void Switch::on_suspend(void *arg)
{
    if(arg == nullptr) {
        if(this->ignore_on_halt) return;

        // set pin to failsafe value
        switch(this->output_type) {
            case DIGITAL: this->digital_pin->set(this->failsafe); break;
            case SIGMADELTA: this->sigmadelta_pin->set(this->failsafe); break;
            case HWPWM: this->pwm_pin->write(switch_value/100.0F); break;
            case SWPWM: this->swpwm_pin->write(switch_value/100.0F); break;
            case NONE: return;
        }
        this->switch_state= this->failsafe;
    }
}
void Switch::on_module_loaded()
{
    this->switch_changed = false;

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_SUSPEND);

    // Settings
    this->on_config_reload(this);
}

// Get config
void Switch::on_config_reload(void *argument)
{
    this->subcode = THEKERNEL->config->value(switch_checksum, this->name_checksum, command_subcode_checksum )->by_default(0)->as_number();
    std::string input_on_command = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_on_command_checksum )->by_default("")->as_string();
    std::string input_off_command = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_off_command_checksum )->by_default("")->as_string();
    this->output_on_command = THEKERNEL->config->value(switch_checksum, this->name_checksum, output_on_command_checksum )->by_default("")->as_string();
    this->output_off_command = THEKERNEL->config->value(switch_checksum, this->name_checksum, output_off_command_checksum )->by_default("")->as_string();
    this->switch_state = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_state_checksum )->by_default(false)->as_bool();

    this->input_pin= new Pin();
    this->input_pin->from_string( THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_checksum )->by_default("nc")->as_string())->as_input();

    bool is_input;

    if(this->input_pin->connected()) {
        std::string ipb = THEKERNEL->config->value(switch_checksum, this->name_checksum, input_pin_behavior_checksum )->by_default("momentary")->as_string();
        this->input_pin_behavior = (ipb == "momentary") ? momentary_checksum : toggle_checksum;
        is_input= true;
        this->ignore_on_halt= true;

    }else{
        delete this->input_pin;
        this->input_pin= nullptr;
        is_input= false;
    }


    if(!is_input) {
        string type = THEKERNEL->config->value(switch_checksum, this->name_checksum, output_type_checksum )->by_default("pwm")->as_string();
        this->failsafe= THEKERNEL->config->value(switch_checksum, this->name_checksum, failsafe_checksum )->by_default(0)->as_number();
        this->ignore_on_halt= THEKERNEL->config->value(switch_checksum, this->name_checksum, ignore_onhalt_checksum )->by_default(false)->as_bool();

        if(type == "pwm"){
            this->output_type= SIGMADELTA;
            this->sigmadelta_pin= new Pwm();
            this->sigmadelta_pin->from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
            if(this->sigmadelta_pin->connected()) {
                if(failsafe == 1) {
                    set_high_on_debug(sigmadelta_pin->port_number, sigmadelta_pin->pin);
                }else{
                    set_low_on_debug(sigmadelta_pin->port_number, sigmadelta_pin->pin);
                }
            }else{
                this->output_type= NONE;
                delete this->sigmadelta_pin;
                this->sigmadelta_pin= nullptr;
            }

        }else if(type == "digital"){
            this->output_type= DIGITAL;
            this->digital_pin= new Pin();
            this->digital_pin->from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
            if(this->digital_pin->connected()) {
                if(failsafe == 1) {
                    set_high_on_debug(digital_pin->port_number, digital_pin->pin);
                }else{
                    set_low_on_debug(digital_pin->port_number, digital_pin->pin);
                }
            }else{
                this->output_type= NONE;
                delete this->digital_pin;
                this->digital_pin= nullptr;
            }

        }else if(type == "hwpwm"){
            this->output_type= HWPWM;
            Pin *pin= new Pin();
            pin->from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
            this->pwm_pin= pin->hardware_pwm();
            if(failsafe == 1) {
                set_high_on_debug(pin->port_number, pin->pin);
            }else{
                set_low_on_debug(pin->port_number, pin->pin);
            }
            delete pin;
            if(this->pwm_pin == nullptr) {
                THEKERNEL->streams->printf("Selected Switch output pin is not PWM capable - disabled");
                this->output_type= NONE;
            }

        }else if(type == "swpwm"){
            this->output_type= SWPWM;
            Pin *pin= new Pin();
            pin->from_string(THEKERNEL->config->value(switch_checksum, this->name_checksum, output_pin_checksum )->by_default("nc")->as_string())->as_output();
            if(pin->connected()) {
                this->swpwm_pin= new SoftPWM(pin, !pin->is_inverting());
                if(failsafe == 1) {
                    set_high_on_debug(pin->port_number, pin->pin);
                }else{
                    set_low_on_debug(pin->port_number, pin->pin);
                }
            }else{
                this->output_type= NONE;
                delete pin;
            }
        } else {
            this->output_type= NONE;
        }

    } else {
        this->output_type= NONE;
        // set to initial state
        this->input_pin_state = this->input_pin->get();
        // input pin polling
        THEKERNEL->slow_ticker->attach( 100, this, &Switch::pinpoll_tick);
    }

    if(!is_input) {
        if(this->output_type == SIGMADELTA) {
            this->sigmadelta_pin->max_pwm(THEKERNEL->config->value(switch_checksum, this->name_checksum, max_pwm_checksum )->by_default(255)->as_number());
            this->switch_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_value_checksum )->by_default(this->sigmadelta_pin->max_pwm())->as_number();
            if(this->switch_state) {
                this->sigmadelta_pin->pwm(this->switch_value); // will be truncated to max_pwm
            } else {
                this->sigmadelta_pin->set(false);
            }

        } else if(this->output_type == HWPWM) {
            // default is 50Hz
            float p= THEKERNEL->config->value(switch_checksum, this->name_checksum, pwm_period_ms_checksum )->by_default(20)->as_number() * 1000.0F; // ms but fractions are allowed
            this->pwm_pin->period_us(p);

            // default is 0% duty cycle
            this->switch_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_value_checksum )->by_default(0)->as_number();
            this->default_on_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, default_on_value_checksum )->by_default(0)->as_number();
            if(this->switch_state) {
                this->pwm_pin->write(this->default_on_value/100.0F);
            } else {
                this->pwm_pin->write(this->switch_value/100.0F);
            }

        } else if(this->output_type == SWPWM) {
            // default is 50Hz
            float p= THEKERNEL->config->value(switch_checksum, this->name_checksum, pwm_period_ms_checksum )->by_default(20)->as_number(); // ms fractions are not allowed
            this->swpwm_pin->period_ms(p);

            // default is 0% duty cycle
            this->switch_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, startup_value_checksum )->by_default(0)->as_number();
            this->default_on_value = THEKERNEL->config->value(switch_checksum, this->name_checksum, default_on_value_checksum )->by_default(0)->as_number();
           if(this->switch_state) {
                this->swpwm_pin->write(this->default_on_value/100.0F);
            } else {
                this->swpwm_pin->write(this->switch_value/100.0F);
            }

        } else if(this->output_type == DIGITAL){
            this->digital_pin->set(this->switch_state);
        }
    }

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


    if(this->output_type == SIGMADELTA) {
        // SIGMADELTA
        THEKERNEL->slow_ticker->attach(1000, this->sigmadelta_pin, &Pwm::on_tick);
    }

    // for commands we need to replace _ for space
    std::replace(output_on_command.begin(), output_on_command.end(), '_', ' '); // replace _ with space
    std::replace(output_off_command.begin(), output_off_command.end(), '_', ' '); // replace _ with space
}

bool Switch::match_input_on_gcode(const Gcode *gcode) const
{
    bool b= ((input_on_command_letter == 'M' && gcode->has_m && gcode->m == input_on_command_code) ||
            (input_on_command_letter == 'G' && gcode->has_g && gcode->g == input_on_command_code));

    return (b && gcode->subcode == this->subcode);
}

bool Switch::match_input_off_gcode(const Gcode *gcode) const
{
    bool b= ((input_off_command_letter == 'M' && gcode->has_m && gcode->m == input_off_command_code) ||
            (input_off_command_letter == 'G' && gcode->has_g && gcode->g == input_off_command_code));
    return (b && gcode->subcode == this->subcode);
}

void Switch::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Add the gcode to the queue ourselves if we need it
    if (!(match_input_on_gcode(gcode) || match_input_off_gcode(gcode))) {
        return;
    }

    // we need to sync this with the queue, so we need to wait for queue to empty, however due to certain slicers
    // issuing redundant swicth on calls regularly we need to optimize by making sure the value is actually changing
    // hence we need to do the wait for queue in each case rather than just once at the start
    if(match_input_on_gcode(gcode)) {
        if (this->output_type == SIGMADELTA) {
            // SIGMADELTA output pin turn on (or off if S0)
            if(gcode->has_letter('S')) {
                int v = roundf(gcode->get_value('S') * sigmadelta_pin->max_pwm() / 255.0F); // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
                if(v != this->sigmadelta_pin->get_pwm()){ // optimize... ignore if already set to the same pwm
                    // drain queue
                    THEKERNEL->conveyor->wait_for_idle();
                    this->sigmadelta_pin->pwm(v);
                    this->switch_state= (v > 0);
                }
            } else {
                // drain queue
                THEKERNEL->conveyor->wait_for_idle();
                this->sigmadelta_pin->pwm(this->switch_value);
                this->switch_state= (this->switch_value > 0);
            }

        } else if (this->output_type == HWPWM) {
            // drain queue
            THEKERNEL->conveyor->wait_for_idle();
            // PWM output pin set duty cycle 0 - 100
            if(gcode->has_letter('S')) {
                float v = gcode->get_value('S');
                if(v > 100) v= 100;
                else if(v < 0) v= 0;
                this->pwm_pin->write(v/100.0F);
                this->switch_state= (ROUND2DP(v) != ROUND2DP(this->switch_value));
            } else {
                this->pwm_pin->write(this->default_on_value/100.0F);
                this->switch_state= true;
            }

       } else if (this->output_type == SWPWM) {
            // drain queue
            THEKERNEL->conveyor->wait_for_idle();
            // PWM output pin set duty cycle 0 - 100
            if(gcode->has_letter('S')) {
                float v = gcode->get_value('S');
                if(v > 100) v= 100;
                else if(v < 0) v= 0;
                this->swpwm_pin->write(v/100.0F);
                this->switch_state= (ROUND2DP(v) != ROUND2DP(this->switch_value));
            } else {
                this->swpwm_pin->write(this->default_on_value/100.0F);
                this->switch_state= true;
            }

        } else if (this->output_type == DIGITAL) {
            // drain queue
            THEKERNEL->conveyor->wait_for_idle();
            // logic pin turn on
            this->digital_pin->set(true);
            this->switch_state = true;
        }

    } else if(match_input_off_gcode(gcode)) {
        // drain queue
        THEKERNEL->conveyor->wait_for_idle();
        this->switch_state = false;
        if (this->output_type == SIGMADELTA) {
            // SIGMADELTA output pin
            this->sigmadelta_pin->set(false);

        } else if (this->output_type == HWPWM) {
            this->pwm_pin->write(this->switch_value/100.0F);

        } else if (this->output_type == SWPWM) {
            this->swpwm_pin->write(this->switch_value/100.0F);

        } else if (this->output_type == DIGITAL) {
            // logic pin turn off
            this->digital_pin->set(false);
        }
    }
}

void Switch::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(switch_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return; // likely fan, but could be anything

    // ok this is targeted at us, so send back the requested data
    // caller has provided the location to write the state to
    struct pad_switch *pad= static_cast<struct pad_switch *>(pdr->get_data_ptr());
    pad->name = this->name_checksum;
    pad->state = this->switch_state;
    pad->value = this->switch_value;
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
        this->switch_changed= true;
        pdr->set_taken();

        // if there is no gcode to be sent then we can do this now (in on_idle)
        // Allows temperature switch to turn on a fan even if main loop is blocked with heat and wait
        if(this->output_on_command.empty() && this->output_off_command.empty()) on_main_loop(nullptr);
    }
}

void Switch::on_main_loop(void *argument)
{
    if(this->switch_changed) {
        if(this->switch_state) {
            if(!this->output_on_command.empty()) this->send_gcode( this->output_on_command, &(StreamOutput::NullStream) );

            if(this->output_type == SIGMADELTA) {
                this->sigmadelta_pin->pwm(this->switch_value); // this requires the value has been set otherwise it switches on to whatever it last was

            } else if (this->output_type == HWPWM) {
                this->pwm_pin->write(this->default_on_value/100.0F);

            } else if (this->output_type == SWPWM) {
                this->swpwm_pin->write(this->default_on_value/100.0F);

            } else if (this->output_type == DIGITAL) {
                this->digital_pin->set(true);
            }

        } else {

            if(!this->output_off_command.empty()) this->send_gcode( this->output_off_command, &(StreamOutput::NullStream) );

            if(this->output_type == SIGMADELTA) {
                this->sigmadelta_pin->set(false);

            } else if (this->output_type == HWPWM) {
                this->pwm_pin->write(this->switch_value/100.0F);

            } else if (this->output_type == SWPWM) {
                this->swpwm_pin->write(this->switch_value/100.0F);

            } else if (this->output_type == DIGITAL) {
                this->digital_pin->set(false);
            }
        }
        this->switch_changed = false;
    }
}

// TODO Make this use InterruptIn
// Check the state of the button and act accordingly
uint32_t Switch::pinpoll_tick(uint32_t dummy)
{
    // If pin changed
    bool current_state = this->input_pin->get();
    if(this->input_pin_state != current_state) {
        this->input_pin_state = current_state;
        // If pin high
        if( this->input_pin_state ) {
            // if switch is a toggle switch
            if( this->input_pin_behavior == toggle_checksum ) {
                this->flip();
            } else {
                // else default is momentary
                this->switch_state = this->input_pin_state;
                this->switch_changed = true;
            }

        } else {
            // else if button released
            if( this->input_pin_behavior == momentary_checksum ) {
                // if switch is momentary
                this->switch_state = this->input_pin_state;
                this->switch_changed = true;
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

