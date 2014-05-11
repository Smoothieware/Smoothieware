/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef thermistor_h
#define thermistor_h

#include "TempSensor.h"
#include "RingBuffer.h"

#define QUEUE_LEN 32


class Thermistor : public TempSensor
{
    public:
        Thermistor();
        ~Thermistor();
        
        // TempSensor interface.
        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
        float get_temperature();
        
    private:
        int new_thermistor_reading();
        float adc_value_to_temperature(int adc_value);

        // Thermistor computation settings
        float r0;
        float t0;
        int r1;
        int r2;
        float beta;
        float j;
        float k;

        Pin  thermistor_pin;

        RingBuffer<uint16_t,QUEUE_LEN> queue;  // Queue of readings
        uint16_t median_buffer[QUEUE_LEN];
        
};

#endif
