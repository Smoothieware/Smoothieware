/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ENCODER_SPINDLE_MODULE_H
#define ENCODER_SPINDLE_MODULE_H

#include "SpindleControl.h"
#include <stdint.h>

namespace mbed {
    class PwmOut;
    class InterruptIn;    
}

class Pin;

// This module implements closed loop PID control for spindle RPM. 
// Currently a clone of PWM module. Will be modified for hardware encoder implementation. 
class EncoderSpindleControl: public SpindleControl {
    public:
        EncoderSpindleControl();
        virtual ~EncoderSpindleControl() {};
        void on_module_loaded();
    
    private:
        
        void on_pin_rise();
        uint32_t on_update_speed(uint32_t dummy);
        uint32_t tick_counter(uint32_t dummy);
        
        Pin *reverse_dir; // digital output pin for reverse
        Pin *switch_on; // digital output pin to Enable spindle drive 
        mbed::PwmOut *pwm_pin; // PWM output for spindle speed control
        mbed::InterruptIn *feedback_pin; // Interrupt pin for measuring speed
        bool output_inverted;
        std::string reverse_dir_pin;
        std::string switch_on_pin;        
               
        bool vfd_spindle; // true if we have a VFD driven spindle

        // Current values, updated at runtime
        float current_rpm;
        float target_rpm;
        float default_rpm;
        float max_rpm;
        float current_I_value;
        float prev_error;
        float current_pwm_value;
        int time_since_update;
        uint32_t last_irq;
       
        // Values from config
        float pulses_per_rev;
        float control_P_term;
        float control_I_term;
        float control_D_term;
        float smoothing_decay;
        float max_pwm;

        // These fields are updated by the interrupt
        uint32_t last_edge; // Timestamp of last edge
        volatile uint32_t last_time; // Time delay between last two edges
        volatile uint32_t irq_count;
        
        void turn_on(void);
        void turn_on_rev(void);
        void turn_off(void);
        void set_speed(int);
        void report_speed(void);
        void set_p_term(float);
        void set_i_term(float);
        void set_d_term(float);
        void report_settings(void);
};

#endif

