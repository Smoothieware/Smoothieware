/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "PWMSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"
#include "utils.h"

#include "libs/Pin.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_pwm_pin_checksum            CHECKSUM("pwm_pin")
#define spindle_pwm_period_checksum         CHECKSUM("pwm_period")
#define spindle_max_pwm_checksum            CHECKSUM("max_pwm")
#define spindle_feedback_pin_checksum       CHECKSUM("feedback_pin")
#define spindle_pulses_per_rev_checksum     CHECKSUM("pulses_per_rev")
#define spindle_default_rpm_checksum        CHECKSUM("default_rpm")
#define spindle_control_P_checksum          CHECKSUM("control_P")
#define spindle_control_I_checksum          CHECKSUM("control_I")
#define spindle_control_D_checksum          CHECKSUM("control_D")
#define spindle_control_smoothing_checksum  CHECKSUM("control_smoothing")

#define UPDATE_FREQ 1000

PWMSpindleControl::PWMSpindleControl()
{
}

void PWMSpindleControl::on_module_loaded()
{
    last_time = 0;
    last_edge = 0;
    current_rpm = 0;
    current_I_value = 0;
    current_pwm_value = 0;
    time_since_update = 0;
    
    spindle_on = false;
    
    pulses_per_rev = THEKERNEL->config->value(spindle_checksum, spindle_pulses_per_rev_checksum)->by_default(1.0f)->as_number();
    target_rpm = THEKERNEL->config->value(spindle_checksum, spindle_default_rpm_checksum)->by_default(5000.0f)->as_number();
    control_P_term = THEKERNEL->config->value(spindle_checksum, spindle_control_P_checksum)->by_default(0.0001f)->as_number();
    control_I_term = THEKERNEL->config->value(spindle_checksum, spindle_control_I_checksum)->by_default(0.0001f)->as_number();
    control_D_term = THEKERNEL->config->value(spindle_checksum, spindle_control_D_checksum)->by_default(0.0001f)->as_number();

    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = THEKERNEL->config->value(spindle_checksum, spindle_control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_pwm_pin_checksum)->by_default("nc")->as_string());
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    max_pwm = THEKERNEL->config->value(spindle_checksum, spindle_max_pwm_checksum)->by_default(1.0f)->as_number();
    
    int period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(1000)->as_int();
    pwm_pin->period_us(period);
    pwm_pin->write(output_inverted ? 1 : 0);

    // Get the pin for interrupt
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_feedback_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            feedback_pin = new mbed::InterruptIn(pinname);
            feedback_pin->rise(this, &PWMSpindleControl::on_pin_rise);
            NVIC_SetPriority(EINT3_IRQn, 16);
        } else {
            THEKERNEL->streams->printf("Error: Spindle feedback pin has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    }
    
    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &PWMSpindleControl::on_update_speed);
}

void PWMSpindleControl::on_pin_rise()
{
    uint32_t timestamp = us_ticker_read();
    last_time = timestamp - last_edge;
    last_edge = timestamp;
    irq_count++;
}

uint32_t PWMSpindleControl::on_update_speed(uint32_t dummy)
{
    // If we don't get any interrupts for 1 second, set current RPM to 0
    uint32_t new_irq = irq_count;
    if (last_irq != new_irq)
        time_since_update = 0;
    else
        time_since_update++;
    last_irq = new_irq;

    if (time_since_update > UPDATE_FREQ)
        last_time = 0;

    // Calculate current RPM
    uint32_t t = last_time;
    if (t == 0) {
        current_rpm = 0;
    } else {
        float new_rpm = 1000000 * 60.0f / (t * pulses_per_rev);
        current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
    }

    if (spindle_on) {
        float error = target_rpm - current_rpm;

        current_I_value += control_I_term * error * 1.0f / UPDATE_FREQ;
        current_I_value = confine(current_I_value, -1.0f, 1.0f);

        float new_pwm = 0.5f;
        new_pwm += control_P_term * error;
        new_pwm += current_I_value;
        new_pwm += control_D_term * UPDATE_FREQ * (error - prev_error);
        new_pwm = confine(new_pwm, 0.0f, 1.0f);
        prev_error = error;

        current_pwm_value = new_pwm;

        if (current_pwm_value > max_pwm) {
            current_pwm_value = max_pwm;
        }
    } else {
        current_I_value = 0;
        current_pwm_value = 0;
    }

    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
    
    return 0;
}

void PWMSpindleControl::turn_on() {
    spindle_on = true;
}

void PWMSpindleControl::turn_off() {
    spindle_on = false;
}


void PWMSpindleControl::set_speed(int rpm) {
    target_rpm = rpm;
}


void PWMSpindleControl::report_speed() {
    THEKERNEL->streams->printf("Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f\n",
                               current_rpm, target_rpm, current_pwm_value);
}


void PWMSpindleControl::set_p_term(float p) {
    control_P_term = p;
}


void PWMSpindleControl::set_i_term(float i) {
    control_I_term = i;
}


void PWMSpindleControl::set_d_term(float d) {
    control_D_term = d;
}


void PWMSpindleControl::report_settings() {
    THEKERNEL->streams->printf("P: %0.6f I: %0.6f D: %0.6f\n",
                               control_P_term, control_I_term, control_D_term);
}

