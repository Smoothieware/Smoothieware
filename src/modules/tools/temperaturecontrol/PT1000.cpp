/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "PT1000.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

#define PT1000_pin_checksum  CHECKSUM("PT1000_pin")

PT1000::PT1000()
{
}

PT1000::~PT1000()
{
}

// Get configuration from the config file
void PT1000::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
	// Pin used for ADC readings
    this->PT1000_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, PT1000_pin_checksum)->required()->as_string());
    THEKERNEL->adc->enable_pin(&PT1000_pin);
}

float PT1000::get_temperature()
{
    float t = adc_value_to_temperature(new_PT1000_reading());
    // keep track of min/max for M305
    if (t > max_temp) max_temp = t;
    if (t < min_temp) min_temp = t;
    return t;
}

void PT1000::get_raw()
{
    int adc_value= new_PT1000_reading();
    float t = adc_value_to_temperature(new_PT1000_reading());
    THEKERNEL->streams->printf("PT1000: adc= %d, temp= %f\n", adc_value, t);
    // reset the min/max
    min_temp = max_temp = t;
}

float PT1000::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
    if ((adc_value >= max_adc_value) || (adc_value == 0))
        return infinityf();

    // polynomial approximation for PT1000, using 4.7kOhm and 1kOhm (PT1000) 3.3V.
    
    float x = (adc_value / (float)max_adc_value);
    float x2 = (x * x);
    float x3 = (x2 * x);
    float x4 = (x3 * x);

    float t = (13980.0f * x4) + (-7019.0f * x3) + (3760.0f * x2) + (796.7f * x) + (-230.9f);

    return t;
}

int PT1000::new_PT1000_reading()
{
    return THEKERNEL->adc->read(&PT1000_pin);
}
