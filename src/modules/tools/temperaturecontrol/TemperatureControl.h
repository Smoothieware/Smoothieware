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

#include "TemperatureControlPublicAccess.h"
#define thermistor_checksum                CHECKSUM("thermistor")
#define r0_checksum                        CHECKSUM("r0")
#define readings_per_second_checksum       CHECKSUM("readings_per_second")
#define max_pwm_checksum                   CHECKSUM("max_pwm")
#define pwm_frequency_checksum             CHECKSUM("pwm_frequency")
#define t0_checksum                        CHECKSUM("t0")
#define beta_checksum                      CHECKSUM("beta")
#define vadc_checksum                      CHECKSUM("vadc")
#define vcc_checksum                       CHECKSUM("vcc")
#define r1_checksum                        CHECKSUM("r1")
#define r2_checksum                        CHECKSUM("r2")
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

#define preset1_checksum                   CHECKSUM("preset1")
#define preset2_checksum                   CHECKSUM("preset2")


#define QUEUE_LEN 8

class TemperatureControlPool;

class TemperatureControl : public Module {

    public:
        TemperatureControl(uint16_t name);

        void on_module_loaded();
        void on_main_loop(void* argument);
        void on_gcode_execute(void* argument);
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);

        void set_desired_temperature(float desired_temperature);
        float get_temperature();
        float adc_value_to_temperature(int adc_value);
        uint32_t thermistor_read_tick(uint32_t dummy);
        int new_thermistor_reading();


        int pool_index;
        TemperatureControlPool *pool;
        friend class PID_Autotuner;

    private:
        void pid_process(float);

        float target_temperature;

        float preset1;
        float preset2;

        // Thermistor computation settings
        float r0;
        float t0;
        int r1;
        int r2;
        float beta;
        float j;
        float k;


        // PID runtime
        float i_max;

        int o;

        float last_reading;

        float acceleration_factor;
        float readings_per_second;

        RingBuffer<uint16_t,QUEUE_LEN> queue;  // Queue of readings
        uint16_t median_buffer[QUEUE_LEN];
        int running_total;

        uint16_t name_checksum;

        Pin  thermistor_pin;
        Pwm  heater_pin;

        bool waiting;
        bool min_temp_violated;

        uint16_t set_m_code;
        uint16_t set_and_wait_m_code;
        uint16_t get_m_code;

        string designator;


        void setPIDp(float p);
        void setPIDi(float i);
        void setPIDd(float d);

        float iTerm;
        float lastInput;
        // PID settings
        float p_factor;
        float i_factor;
        float d_factor;
        float PIDdt;
};

#endif
