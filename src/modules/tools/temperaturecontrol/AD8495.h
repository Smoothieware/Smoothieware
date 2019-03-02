/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef AD8495_H
#define AD8495_H

#include "TempSensor.h"
#include "RingBuffer.h"
#include "Pin.h"

#include <tuple>

#define QUEUE_LEN 32

class StreamOutput;

class AD8495 : public TempSensor
{
    public:
        AD8495();
        ~AD8495();

        // TempSensor interface.
        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
        float get_temperature();
        void get_raw();

    private:
        int new_AD8495_reading();
        float adc_value_to_temperature(uint32_t adc_value);

        Pin  AD8495_pin;
        float AD8495_offset;
        
        float min_temp, max_temp;
};

#endif
