/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef max31865_h
#define max31865_h

#include "TempSensor.h"
#include <atomic>
#include <string>
#include <libs/Pin.h>
#include <mbed.h>

class Max31865 : public TempSensor
{
public:
    Max31865();
    ~Max31865();
    void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
    float get_temperature();
    void get_raw();
    void on_idle();

private:
    void init_rtd();
    float adc_value_to_resistance(uint16_t adcValue);
    float resistance_to_temperature(float Rt);
    void print_errors(bool always = false);

    uint8_t read_register_8(uint8_t reg);
    uint16_t read_register_16(uint8_t reg);
    void write_register_8(uint8_t reg, uint8_t val);

    Pin spi_cs_pin;
    mbed::SPI *spi;

    float rtd_nominal_resistance;
    float reference_resistor;
    bool use_50hz_filter;

    uint32_t last_reading_time;
    std::atomic<float> last_temperature;

    uint8_t errors_reported;
};

#endif
