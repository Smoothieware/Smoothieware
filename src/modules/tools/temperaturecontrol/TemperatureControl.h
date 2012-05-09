/*  
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef temperaturecontrol_h
#define temperaturecontrol_h

#include "libs/Pin.h"
#include <math.h>

#define UNDEFINED -1

#define thermistor_checksum                41045
#define r0_checksum                        5538
#define readings_per_second_checksum       18645 
#define t0_checksum                        6564
#define beta_checksum                      1181
#define vadc_checksum                      10911
#define vcc_checksum                       36157
#define r1_checksum                        5795
#define r2_checksum                        6052
#define temperature_control_checksum       44054
#define thermistor_pin_checksum            1788 
#define heater_pin_checksum                35619 

class TemperatureControl : public Module {
    public:
        TemperatureControl();
        TemperatureControl(uint16_t name);
        
        void on_module_loaded();
        void on_main_loop(void* argument);
        void on_gcode_execute(void* argument);
        void on_config_reload(void* argument);
        void set_desired_temperature(double desired_temperature);
        double get_temperature();
        double adc_value_to_temperature(double adc_value);
        double temperature_to_adc_value(double temperature);
        uint32_t thermistor_read_tick(uint32_t dummy);
        double new_thermistor_reading();
        double average_adc_reading();
        
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

        uint16_t name_checksum;

        Pin* thermistor_pin;
        Pin* heater_pin;
    
        bool waiting;

};

#endif
