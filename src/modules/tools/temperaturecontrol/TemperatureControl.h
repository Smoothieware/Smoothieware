#ifndef TEMPERATURECONTROL_H
#define TEMPERATURECONTROL_H

#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>

#define UNDEFINED -1

class TemperatureControl : public Module {
    public:
        TemperatureControl();
        
        void on_module_loaded();
        void on_gcode_execute(void* argument);
        void set_desired_temperature(double desired_temperature);
        double get_temperature();
        double adc_value_to_temperature(double adc_value);
        double temperature_to_adc_value(double temperature);
        void thermistor_read_tick();
        double new_thermistor_reading();
        double average_adc_reading();
        
    
        AnalogIn* thermistor_pin;
        PwmOut*   heater_pwm;
        double    pwm_value; 
        double    desired_adc_value;
        double    tail_adc_value;
        double    head_adc_value;

        // Thermistor computation settings
        double r0;
        double t0;
        double beta;
        double vadc;
        double vcc;
        double k;
        double vs;
        double rs;
        
        double acceleration_factor;
        double readings_per_second;

        RingBuffer<double,16> queue;  // Queue of Blocks
        int error_count;
};

#endif
