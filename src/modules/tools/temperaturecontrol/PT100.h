#ifndef PT100_H
#define PT100_H

#include "TempSensor.h"
#include "Pin.h"

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
    void get_raw(); //called from M305

private:
    int get_adc_reading();
    float adc_value_to_temperature(uint32_t adc_value);
    Pin amplifier_pin;
    float min_temp, max_temp;
    float baseline_temp;
    int baseline_adc_value;
    int adc_delta_deg;
};
#endif