/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperaturecontrol_h
#define temperaturecontrol_h

#include "Module.h"
#include "Pwm.h"
#include "TempSensor.h"
#include "TemperatureControlPublicAccess.h"

class TemperatureControl : public Module {

    public:
        TemperatureControl(uint16_t name, int index);
        ~TemperatureControl();

        void on_module_loaded();
        void on_main_loop(void* argument);
        void on_gcode_execute(void* argument);
        void on_gcode_received(void* argument);
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_halt(void* argument);

        void set_desired_temperature(float desired_temperature);

        float get_temperature();

        friend class PID_Autotuner;

    private:
        void load_config();
        uint32_t thermistor_read_tick(uint32_t dummy);
        void pid_process(float);

        int pool_index;

        float target_temperature;

        float preset1;
        float preset2;

        TempSensor *sensor;

        // PID runtime
        float i_max;

        int o;

        float last_reading;

        float readings_per_second;

        uint16_t name_checksum;

        Pwm  heater_pin;

        struct {
            bool use_bangbang:1;
            bool waiting:1;
            bool min_temp_violated:1;
            bool link_to_tool:1;
            bool active:1;
            bool readonly:1;
        };

        uint16_t set_m_code;
        uint16_t set_and_wait_m_code;
        uint16_t get_m_code;
        struct pad_temperature public_data_return;

        std::string designator;

        void setPIDp(float p);
        void setPIDi(float i);
        void setPIDd(float d);

        float hysteresis;
        float iTerm;
        float lastInput;
        // PID settings
        float p_factor;
        float i_factor;
        float d_factor;
        float PIDdt;
};

#endif
