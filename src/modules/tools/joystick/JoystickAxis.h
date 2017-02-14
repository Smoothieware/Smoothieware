/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef JOYSTICKAXIS_H
#define JOYSTICKAXIS_H

#include "Pin.h"
#include "Module.h"

#define ADC_VREF 3.3f

class JoystickAxis : public Module {
    public:
        JoystickAxis();
        JoystickAxis(uint16_t name);

        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_get_public_data(void* argument);
        uint32_t update_tick(uint32_t);

    private:
        uint16_t name_checksum{ 0 }; //the "name" of this instance (only a checksum of the text however)

        float position{ 0 }; //the scaled axis value (-1 to 1)

        Pin axis_pin; //which pin on the smoothie will provide the analog input
        float zero_offset{ 0 }; //the voltage which corresponds to zero on the scaled axis
        float endpoint{ ADC_VREF }; //the voltage which corresponds to either 1 or -1 (depends on its size relative to zero_offset)

        bool auto_zero{ false }; //whether or not to automatically determine the zero-offset at startup
        bool in_startup{ true }; //whether the module is currently in startup mode
        float last_reading{ 0.0f }; //stores the last ADC reading
        int startup_time{ 1000 }; //number of milliseconds to spend determining the zero-offset before normal functionality
        int startup_intervals{ 0 }; //keeps track of how many refresh intervals have passed in the startup time
        float startup_sum{ 0.0f }; //keeps track of the sum of ADC readings during startup

        int refresh_rate{ 10 }; //number of ADC readings per second (absolute max = base_stepping_frequency (100 kHz))

        float read_pos(); //read the ADC voltage (0 to ADC_VREF)
        float get_normalized(float pos); //get the scaled value (-1 to 1) from the ADC value
};

#endif //JOYSTICKAXIS_H