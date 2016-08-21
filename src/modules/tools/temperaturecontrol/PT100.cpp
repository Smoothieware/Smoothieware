/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "PT100.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "libs/Median.h"
#include "utils.h"
#include "StreamOutputPool.h"

#include <fastmath.h>

#include "MRI_Hooks.h"

#define UNDEFINED -1

#define PT100_pin_checksum          CHECKSUM("PT100_pin")
#define PT100_current_checksum      CHECKSUM("PT100_current")
#define PT100_gain_checksum         CHECKSUM("PT100_gain")
#define designator_checksum                CHECKSUM("designator")
PT100::PT100()
{
	min_temp = 999;
    max_temp= 0;
}

PT100::~PT100()
{
}

// Get configuration from the config file
void PT100::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
	std::string s = THEKERNEL->config->value(module_checksum, name_checksum, designator_checksum)->required()->as_string();
/*	if(s[0] == 'T')
		this->PT100_pin.from_string("P0_23");
	else if(s[0] == 'B')
		this->PT100_pin.from_string("P0_26");
	else if(s[0] == 'C')
			this->PT100_pin.from_string("P0_25");
	else
		THEKERNEL->streams->printf("Aquecedor nao reconhecido!\r\n");*/
    this->PT100_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, PT100_pin_checksum)->required()->as_string());
    this->PT100_current = THEKERNEL->config->value(module_checksum, name_checksum, PT100_current_checksum)->by_default((double)0.001)->as_double();
	this->PT100_gain = THEKERNEL->config->value(module_checksum, name_checksum, PT100_gain_checksum)->by_default(10)->as_number(); 
    THEKERNEL->adc->enable_pin(&PT100_pin);
}


float PT100::get_temperature()
{
    float t= adc_value_to_temperature(new_PT100_reading());
    // keep track of min/max for M305
    if(t > max_temp) max_temp= t;
    if(t < min_temp) min_temp= t;
    return t;
}

void PT100::get_raw()
{

    int adc_value= new_PT100_reading();
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
	double t = ((double)adc_value) *(3.3 / (double)max_adc_value);
	t = (((((t/this->PT100_gain)/this->PT100_current) / 100) - 1) / 0.00385);

    THEKERNEL->streams->printf("adc= %d, max_adc= %lu, temp= %f, current = %f\n", adc_value,max_adc_value,t, this->PT100_current);

    // reset the min/max
    min_temp= max_temp= (float) t;
}

float PT100::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
    if ((adc_value >= max_adc_value))
        return infinityf();
	//converte adc para tensao
    double t=((double)adc_value) *( 3.3 / (double)max_adc_value);
	t = (((((t / this->PT100_gain) / this->PT100_current) / 100) - 1) / 0.00385);
    return (float) t;
}

int PT100::new_PT100_reading()
{
    // filtering now done in ADC
    return THEKERNEL->adc->read(&PT100_pin);
}
