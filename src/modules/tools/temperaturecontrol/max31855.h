/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef max31855_h
#define max31855_h

#include "TempSensor.h"
#include <string>
#include <libs/Pin.h>
#include <mbed.h>
#include "RingBuffer.h"

class Max31855 : public TempSensor
{
public:
    Max31855();
    ~Max31855();
    void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
    float get_temperature();
    void on_idle();

private:
    struct { bool read_flag:1; } ; //when true, the next call to on_idle will read a new temperature value
    Pin spi_cs_pin;
    mbed::SPI *spi;
    RingBuffer<float,16> readings;
};

#endif
