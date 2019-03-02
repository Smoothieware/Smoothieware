/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "PT100_E3D.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

#define e3d_amplifier_pin_checksum  CHECKSUM("e3d_amplifier_pin")

PT100_E3D::PT100_E3D()
{
}

PT100_E3D::~PT100_E3D()
{
}

// Get configuration from the config file
void PT100_E3D::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
	// Pin used for ADC readings
    this->amplifier_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, e3d_amplifier_pin_checksum)->required()->as_string());
    THEKERNEL->adc->enable_pin(&amplifier_pin);
}

float PT100_E3D::get_temperature()
{
    float t = adc_value_to_temperature(new_pt100_reading());
    // keep track of min/max for M305
    if (t > max_temp) max_temp = t;
    if (t < min_temp) min_temp = t;
    return t;
}

void PT100_E3D::get_raw()
{
    int adc_value= new_pt100_reading();
    float t = adc_value_to_temperature(new_pt100_reading());
    THEKERNEL->streams->printf("PT100_E3D: adc= %d, temp= %f\n", adc_value, t);
    // reset the min/max
    min_temp = max_temp = t;
}

float PT100_E3D::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
    if ((adc_value >= max_adc_value) || (adc_value == 0))
        return infinityf();

    // polynomial approximation of E3D published curve, using normalized ADC values instead of voltages
    float x = (adc_value / (float)max_adc_value);
    float x2 = (x * x);
    float t = (382.7f * x2) + (1004.8f * x) - 241.84f;

    return t;
}

int PT100_E3D::new_pt100_reading()
{
    return THEKERNEL->adc->read(&amplifier_pin);
}
