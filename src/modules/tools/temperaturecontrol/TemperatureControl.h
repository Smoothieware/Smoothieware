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

#define thermistor_checksum                CHECKSUM("thermistor")
#define r0_checksum                        CHECKSUM("r0")
#define readings_per_second_checksum       CHECKSUM("readings_per_second")
#define max_pwm_checksum                   CHECKSUM("max_pwm")
#define t0_checksum                        CHECKSUM("t0")
#define beta_checksum                      CHECKSUM("beta")
#define vadc_checksum                      CHECKSUM("vadc")
#define vcc_checksum                       CHECKSUM("vcc")
#define r1_checksum                        CHECKSUM("r1")
#define r2_checksum                        CHECKSUM("r2")
#define temperature_control_checksum       CHECKSUM("temperature_control")
#define thermistor_pin_checksum            CHECKSUM("thermistor_pin")
#define heater_pin_checksum                CHECKSUM("heater_pin")

#define get_m_code_checksum                CHECKSUM("get_m_code")
#define set_m_code_checksum                CHECKSUM("set_m_code")
#define set_and_wait_m_code_checksum       CHECKSUM("set_and_wait_m_code")

#define designator_checksum                CHECKSUM("designator")

#define p_factor_checksum                  CHECKSUM("p_factor")
#define i_factor_checksum                  CHECKSUM("i_factor")
#define d_factor_checksum                  CHECKSUM("d_factor")

#define i_max_checksum                     CHECKSUM("i_max")

#define QUEUE_LEN 8

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

        int max_pwm;

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

        RingBuffer<uint16_t,QUEUE_LEN> queue;  // Queue of readings
        uint16_t median_buffer[QUEUE_LEN];
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
