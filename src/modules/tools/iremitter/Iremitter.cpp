/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "Iremitter.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "StreamOutputPool.h"

#include "libs/Pin.h"
#include "PwmOut.h" // mbed.h lib

#include "predefined_ir_codes.h"

// config names
#define iremitter_checksum CHECKSUM("iremitter")
#define enable_checksum    CHECKSUM("enable")
#define led_pin_checksum   CHECKSUM("led_pin")
#define m_code_checksum    CHECKSUM("m_code")
#define device_checksum    CHECKSUM("device")
#define action_checksum    CHECKSUM("action")
#define frequency_checksum CHECKSUM("frequency")
#define raw_code_checksum  CHECKSUM("raw_code")

Iremitter::Iremitter() {}

void Iremitter::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(! THEKERNEL->config->value(iremitter_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // setup initial settings
    this->on_config_reload(this);

    // register for main events
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
}

void Iremitter::on_config_reload(void *argument)
{
    // dummy pin to check harware PWM
    Pin* dummy_pin = new Pin();
    dummy_pin->from_string(THEKERNEL->config->value(iremitter_checksum, led_pin_checksum)->by_default("nc")->as_string());

    // if available, return mbed hardware pwm class for this pin
    this->led_pin = dummy_pin->as_output()->hardware_pwm();

    // hardware pwm not enabled on this pin
    if (this->led_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Iremitter cannot use P%d.%d (P2.[0-5], P1.[18,20,21,23,24,26], P3.[25,26] only), module disabled.\n", dummy_pin->port_number, dummy_pin->pin);
        delete dummy_pin;
        delete this;
        return;
    }

    // free up the resource
    delete dummy_pin;
    dummy_pin = NULL;

    // M code as integer
    this->m_code = THEKERNEL->config->value(iremitter_checksum, m_code_checksum)->by_default(0)->as_int();

    // device and action name
    string device = THEKERNEL->config->value(iremitter_checksum, device_checksum)->by_default("nikon")->as_string();
    string action = THEKERNEL->config->value(iremitter_checksum, action_checksum)->by_default("trigger_now")->as_string();

    // predefined code ?
    bool found = false;
    
    for (auto& i : predefined_ir_codes) {
        if(device.compare(i.device) == 0 && action.compare(i.action) == 0) 
        {
            // device/action couple found, take code and frequency
            this->raw_code  = this->split_raw_code(i.raw_code, ',');
            this->frequency = i.frequency;
            found = true;
            break;
        }
    }

    // user raw code ?
    if (!found)
    {
        this->frequency      = THEKERNEL->config->value(iremitter_checksum, frequency_checksum)->by_default(40)->as_int();
        string user_raw_code = THEKERNEL->config->value(iremitter_checksum, raw_code_checksum)->by_default("")->as_string();
        this->raw_code       = this->split_raw_code(user_raw_code.c_str(), ',');
    }

    // PWM frequency and period
    this->led_pin->period_us(1000 / this->frequency);
    
    // set the pin LOW
    this->led_pin->write(0.00F);
    this->led_pin_high = false;

    // ir code pointer
    this->raw_code_ptr = 0;
}

// split raw ir code on commas, and return an vector of integer
vector<int> Iremitter::split_raw_code(const char *str, char c)
{
    vector<int> result;

    do {
        const char *begin = str;

        while(*str != c && *str)
            str++;

        result.push_back(atoi(string(begin, str).c_str()));
    } while (0 != *str++);

    return result;
}

bool Iremitter::is_executable(Gcode *gcode)
{
    // return true if all condition for execution is satisfied...
    return this->m_code > 0 and gcode->has_m and gcode->m == this->m_code;
}

void Iremitter::on_gcode_received(void *argument)
{
    // received gcode
    Gcode *gcode = static_cast<Gcode *>(argument);

    // if executable, append to the queue
    if (this->is_executable(gcode))
        THEKERNEL->conveyor->append_gcode(gcode);
}

void Iremitter::on_gcode_execute(void *argument)
{
    // executed gcode
    Gcode *gcode = static_cast<Gcode *>(argument);

    // if executable, trigger ir code
    if (this->is_executable(gcode)) 
        this->trigger();
}

void Iremitter::trigger_loop()
{
    // not the first call
    if (this->raw_code_ptr > 0) 
    {
        // detach the timeout
        this->timeout.detach();

        // out of range, code end.
        if (this->raw_code_ptr >= this->raw_code.size()) 
        {
            // turn off the led
            this->led_pin->write(0.00F);

            // reset pointer and led status
            this->led_pin_high = false;
            this->raw_code_ptr  = 0;

            // exit...
            return;
        }
    }

    // toggle the led status
    this->led_pin_high = !this->led_pin_high;
    if (this->led_pin_high) this->led_pin->write(0.50F);
    else this->led_pin->write(0.00F);

    // timeout for this pulse
    int timeout = this->raw_code[this->raw_code_ptr];

    // set the next timeout to toggle the led status
    this->timeout.attach_us(this, &Iremitter::trigger_loop, timeout);
    
    // increment pointer
    this->raw_code_ptr += 1;
}

void Iremitter::trigger()
{
    // launch trigger loop, if not allready.
    if (this->raw_code_ptr == 0)
        this->trigger_loop();
}

