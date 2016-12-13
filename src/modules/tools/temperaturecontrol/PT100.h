#ifndef PT100_H
#define PT100_H

#include "TempSensor.h"
#include "RingBuffer.h"
#include "Pin.h"

#include <tuple>

#define QUEUE_LEN 32

class StreamOutput;

class PT100 : public TempSensor
{
    public:
        PT100();
        ~PT100();

        // TempSensor interface.
        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
        float get_temperature();
        void get_raw();

    private:
        int new_PT100_reading();
        float adc_value_to_temperature(uint32_t adc_value);

        Pin  PT100_pin;
        double PT100_current;
	    float PT100_gain;
        float min_temp, max_temp;
};

#endif
