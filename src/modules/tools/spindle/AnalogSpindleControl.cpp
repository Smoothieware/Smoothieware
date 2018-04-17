/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "AnalogSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "PwmOut.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_max_rpm_checksum            CHECKSUM("max_rpm")
#define spindle_min_rpm_checksum            CHECKSUM("min_rpm")
#define spindle_pwm_pin_checksum            CHECKSUM("pwm_pin")
#define spindle_pwm_period_checksum         CHECKSUM("pwm_period")
#define spindle_switch_on_pin_checksum      CHECKSUM("switch_on_pin")

void AnalogSpindleControl::on_module_loaded()
{

    spindle_on = false;
    target_rpm = 0;
    min_rpm = THEKERNEL->config->value(spindle_checksum, spindle_min_rpm_checksum)->by_default(100)->as_int();
    max_rpm = THEKERNEL->config->value(spindle_checksum, spindle_max_rpm_checksum)->by_default(5000)->as_int();

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_pwm_pin_checksum)->by_default("nc")->as_string());
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    // If we got no hardware PWM pin, delete this module
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }
    
    // set pwm frequency
    int period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(1000)->as_int();
    pwm_pin->period_us(period);
    // invert pwm signal if necessary
    pwm_pin->write(output_inverted ? 1 : 0);

    // Get digital out pin for switching the VFD on and off (wired to a digital input on the VFD via an optocoupler)
    std::string switch_on_pin = THEKERNEL->config->value(spindle_checksum, spindle_switch_on_pin_checksum)->by_default("nc")->as_string();
    switch_on = NULL;
    if(switch_on_pin.compare("nc") != 0) {
        switch_on = new Pin();
        switch_on->from_string(switch_on_pin)->as_output()->set(false);
    }
}

void AnalogSpindleControl::turn_on() 
{
    // set the output for switching the VFD on
    if(switch_on != NULL) 
        switch_on->set(true); 
    spindle_on = true;

}


void AnalogSpindleControl::turn_off() 
{
    // clear the output for switching the VFD on 
    if(switch_on != NULL) 
        switch_on->set(false);
    spindle_on = false;
    // set the PWM value to 0 to make sure it stops
    update_pwm(0);

}


void AnalogSpindleControl::set_speed(int rpm) 
{
    // limit the requested RPM value
    if(rpm < 0) {
        target_rpm = 0;
    } else if (rpm > max_rpm) {
        target_rpm = max_rpm;
    } else if (rpm > 0 && rpm < min_rpm){
        target_rpm = min_rpm;
    } else {
        target_rpm = rpm;
    }
    // calculate the duty cycle and update the PWM
    update_pwm(1.0f / max_rpm * target_rpm);

}


void AnalogSpindleControl::report_speed() 
{
    // report the current PWM value, calculate the current RPM value and report it as well
    float current_pwm = pwm_pin->read();
    THEKERNEL->streams->printf("Current RPM: %.0f Analog value: %5.3f Target RPM: %d\n",
                               max_rpm * current_pwm, current_pwm, target_rpm);

}


void AnalogSpindleControl::update_pwm(float value) 
{
    // set the requested PWM value, invert it if necessary
    if(output_inverted)
        pwm_pin->write(1.0f - value);
    else
        pwm_pin->write(value);

}

