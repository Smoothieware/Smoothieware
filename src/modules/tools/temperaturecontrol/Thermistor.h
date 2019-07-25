/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "TempSensor.h"
#include "RingBuffer.h"
#include "Pin.h"

#include <tuple>

#define QUEUE_LEN 32

struct thermistor_t
{
	const char *name;
    // on board resistor settings
	int r1;
	int r2;
    // this saves memory as we only use either beta or SHH
	union {
		struct {
			float c1;
			float c2;
			float c3;
		};
		struct {
			float beta;
			float r0;
			float t0;
		};
		struct {
			float cc0;
			float cc1;
			float cc2;
		};
	};
	float(*calc_fn)(thermistor_t *, float);
};

class StreamOutput;

class Thermistor : public TempSensor
{
    public:
        Thermistor();
        ~Thermistor();

        // TempSensor interface.
        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
        float get_temperature();
        bool set_optional(const sensor_options_t& options);
        bool get_optional(sensor_options_t& options);
        void get_raw();
        static std::tuple<float,float,float> calculate_steinhart_hart_coefficients(float t1, float r1, float t2, float r2, float t3, float r3);
        static void print_predefined_thermistors(StreamOutput*);

    private:
        int new_thermistor_reading();
        float adc_value_to_temperature(uint32_t adc_value);
        Pin  thermistor_pin;

        float min_temp, max_temp;
        bool bad_config;
        uint8_t thermistor_number;
        thermistor_t thermistor;
};

#endif
