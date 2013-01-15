/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperaturecontrol_h
#define temperaturecontrol_h

#include "libs/Pin.h"
#include "Pwm.h"
#include <math.h>

#include "RingBuffer.h"

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

#define get_m_code_checksum                20746
#define set_m_code_checksum                51478
#define set_and_wait_m_code_checksum       4287

#define designator_checksum                49716

#define p_factor_checksum                   43089
#define i_factor_checksum                   28746
#define d_factor_checksum                   18501

#define i_max_checksum                      4112

class TemperatureControlPool;

class TemperatureControl : public Module {
    public:
        TemperatureControl();
        TemperatureControl(uint16_t name);

        void on_module_loaded();
        void on_main_loop(void* argument);
        void on_gcode_execute(void* argument);
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);
        void on_second_tick(void* argument);

        void set_desired_temperature(double desired_temperature);
        double get_temperature();
        double adc_value_to_temperature(int adc_value);
        uint32_t thermistor_read_tick(uint32_t dummy);
        int new_thermistor_reading();

        void pid_process(double);

        double target_temperature;

        // Thermistor computation settings
        double r0;
        double t0;
        int r1;
        int r2;
        double beta;
        double j;
        double k;

        // PID settings
        double p_factor;
        double i_factor;
        double d_factor;

        // PID runtime
        double i_max;

        double p, i, d;
        int o;

        double last_reading;

        double acceleration_factor;
        double readings_per_second;

        RingBuffer<uint16_t,8> queue;  // Queue of readings
        int running_total;

        uint16_t name_checksum;

        Pin* thermistor_pin;
        Pwm* heater_pin;

        bool waiting;

        uint16_t set_m_code;
        uint16_t set_and_wait_m_code;
        uint16_t get_m_code;

        string designator;

        TemperatureControlPool *pool;
        int pool_index;
};

#endif
