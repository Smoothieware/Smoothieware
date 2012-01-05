#ifndef TEMPERATURECONTROL_H
#define TEMPERATURECONTROL_H

#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>

#define UNDEFINED -1

#define temperature_control_thermistor_checksum 22986
#define temperature_control_r0_ckeckusm    8728
#define readings_per_second_ckeckusm       18645 
#define temperature_control_t0_ckeckusm    9754
#define temperature_control_beta_ckeckusm  64275 
#define temperature_control_vadc_ckeckusm  8725
#define temperature_control_vcc_ckeckusm   4274
#define temperature_control_r1_ckeckusm    8985
#define temperature_control_r2_ckeckusm    9242




class TemperatureControl : public Module {
    public:
        TemperatureControl();
        
        void on_module_loaded();
        void on_gcode_execute(void* argument);
        void on_config_reload(void* argument);
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
        double r1;
        double r2;
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
