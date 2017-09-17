/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PT100_E3D_H
#define PT100_E3D_H

#include "TempSensor.h"
#include "Pin.h"

// PT100 sensor via E3D amplifier
class PT100_E3D : public TempSensor
{
public:
	PT100_E3D();
	~PT100_E3D();

	// TempSensor interface.
	void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);
	float get_temperature();
	void get_raw();

private:
    int new_pt100_reading();
    float adc_value_to_temperature(uint32_t adc_value);

	Pin amplifier_pin;
    float min_temp, max_temp;
};

#endif
