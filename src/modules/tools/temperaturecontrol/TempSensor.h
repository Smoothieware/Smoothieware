/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H

#include <map>
#include <stdint.h>

class TempSensor
{
public:
    virtual ~TempSensor() {}

    // Load config parameters using provided "base" names.
    virtual void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum) {}

    // Return temperature in degrees Celsius.
    virtual float get_temperature() { return -1.0F; }

    typedef std::map<char, float> sensor_options_t;
    virtual bool set_optional(const sensor_options_t& options) { return false; }
    virtual bool get_optional(sensor_options_t& options) { return false; }
    virtual void get_raw() {}
    virtual void on_idle() {}
};

#endif
