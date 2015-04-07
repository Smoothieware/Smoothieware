/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Spindle.h"
#include "Config.h"
#include "libs/nuts_bolts.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"

#include "libs/Pin.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"

#define spindle_enable_checksum          CHECKSUM("spindle_enable")
#define spindle_pwm_pin_checksum         CHECKSUM("spindle_pwm_pin")
#define spindle_pwm_period_checksum      CHECKSUM("spindle_pwm_period")
#define spindle_feedback_pin_checksum    CHECKSUM("spindle_feedback_pin")
#define spindle_pulses_per_rev_checksum  CHECKSUM("spindle_pulses_per_rev")
#define spindle_default_rpm_checksum     CHECKSUM("spindle_default_rpm")
#define spindle_control_P_checksum       CHECKSUM("spindle_control_P")
#define spindle_control_I_checksum       CHECKSUM("spindle_control_I")
#define spindle_control_D_checksum       CHECKSUM("spindle_control_D")
#define spindle_control_smoothing_checksum CHECKSUM("spindle_control_smoothing")

#define UPDATE_FREQ 1000

Spindle::Spindle()
{
}

void Spindle::on_module_loaded()
{
    last_time = 0;
    last_edge = 0;
    current_rpm = 0;
    current_I_value = 0;
    current_pwm_value = 0;
    time_since_update = 0;
    spindle_on = true;
    
    if (!THEKERNEL->config->value(spindle_enable_checksum)->by_default(false)->as_bool())
    {
      delete this; // Spindle control module is disabled
      return;
    }

    pulses_per_rev = THEKERNEL->config->value(spindle_pulses_per_rev_checksum)->by_default(1.0f)->as_number();
    target_rpm = THEKERNEL->config->value(spindle_default_rpm_checksum)->by_default(5000.0f)->as_number();
    control_P_term = THEKERNEL->config->value(spindle_control_P_checksum)->by_default(0.0001f)->as_number();
    control_I_term = THEKERNEL->config->value(spindle_control_I_checksum)->by_default(0.0001f)->as_number();
    control_D_term = THEKERNEL->config->value(spindle_control_D_checksum)->by_default(0.0001f)->as_number();
    
    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = THEKERNEL->config->value(spindle_control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);
    
    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_pwm_pin_checksum)->by_default("nc")->as_string());
        spindle_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->inverting;
        delete smoothie_pin;
    }
    
    if (spindle_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }
    
    int period = THEKERNEL->config->value(spindle_pwm_period_checksum)->by_default(1000)->as_int();
    spindle_pin->period_us(period);
    spindle_pin->write(output_inverted ? 1 : 0);
    
    // Get the pin for interrupt
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_feedback_pin_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2)
        {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            feedback_pin = new mbed::InterruptIn(pinname);
            feedback_pin->rise(this, &Spindle::on_pin_rise);
            NVIC_SetPriority(EINT3_IRQn, 16);
        }
        else
        {
            THEKERNEL->streams->printf("Error: Spindle feedback pin has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    }
    
    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &Spindle::on_update_speed);
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GCODE_EXECUTE);
}

void Spindle::on_pin_rise()
{
    uint32_t timestamp = us_ticker_read();
    last_time = timestamp - last_edge;
    last_edge = timestamp;
    irq_count++;
}

uint32_t Spindle::on_update_speed(uint32_t dummy)
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
    if (t == 0)
    {
        current_rpm = 0;
    }
    else
    {
        float new_rpm = 1000000 * 60.0f / (t * pulses_per_rev);
        current_rpm = smoothing_decay * new_rpm + (1.0f - smoothing_decay) * current_rpm;
    }
    
    if (spindle_on)
    {
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
    }
    else
    {
        current_I_value = 0;
        current_pwm_value = 0;
    }
    
    if (output_inverted)
        spindle_pin->write(1.0f - current_pwm_value);
    else
        spindle_pin->write(current_pwm_value);
    
    return 0;
}


void Spindle::on_gcode_received(void* argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    
    if (gcode->has_m)
    {
        if (gcode->m == 957)
        {
            // M957: report spindle speed
            THEKERNEL->streams->printf("Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f\n",
                                       current_rpm, target_rpm, current_pwm_value);
            gcode->mark_as_taken();
        }
        else if (gcode->m == 958)
        {
            // M958: set spindle PID parameters
            if (gcode->has_letter('P'))
                control_P_term = gcode->get_value('P');
            if (gcode->has_letter('I'))
                control_I_term = gcode->get_value('I');
            if (gcode->has_letter('D'))
                control_D_term = gcode->get_value('D');
            THEKERNEL->streams->printf("P: %0.6f I: %0.6f D: %0.6f\n",
                control_P_term, control_I_term, control_D_term);
        }
        else if (gcode->m == 3 || gcode->m == 5)
        {
            // M3: Spindle on, M5: Spindle off
            THEKERNEL->conveyor->append_gcode(gcode);
            gcode->mark_as_taken();
        }
    }
}

void Spindle::on_gcode_execute(void* argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    
    if (gcode->has_m)
    {
        if (gcode->m == 3)
        {
            // M3: Spindle on
            spindle_on = true;
            
            if (gcode->has_letter('S'))
            {
                target_rpm = gcode->get_value('S');
            }
        }
        else if (gcode->m == 5)
        {
            spindle_on = false;
        }
    }
}

