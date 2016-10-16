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
        void get_raw();

    private:
        int new_thermistor_reading();
        float adc_value_to_temperature(uint32_t adc_value);
        Pin  thermistor_pin;
        Pin  ampmod1_pin;
        Pin  ampmod2_pin;
        float min_temp, max_temp;
	float m, b; //reuse m as A if not linear
        float r0;
        float q1,q2,q3;
	unsigned char amptype;
        struct {
            bool bad_config:1;
            bool use_ampmod1:1;
            bool use_ampmod2:1;
            bool use_linear:1;
        };
};

#endif
