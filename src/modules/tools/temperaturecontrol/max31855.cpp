/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include <math.h>
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "max31855.h"

#include "MRI_Hooks.h"

#define chip_select_checksum CHECKSUM("chip_select")
#define spi_channel_checksum CHECKSUM("spi_channel")

Max31855::Max31855()
{
}

Max31855::~Max31855()
{
}

// Get configuration from the config file
void Max31855::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
	// Chip select
    this->spi_cs_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, chip_select_checksum)->by_default("nc")->as_string())->as_output();
	this->spi_cs_pin.set(true);
	
    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(module_checksum, name_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName miso;
    PinName mosi;
    PinName sclk;
    if(spi_channel == 0) {
		// Channel 0
        mosi=P0_18; miso=P0_17; sclk=P0_15;
    } else {
		// Channel 1
        mosi=P0_9; miso=P0_8; sclk=P0_7;
    } 

	spi.reset(new SPI(mosi, miso, sclk));

	this->spi->format(32);
    this->spi->frequency(1000000);
}

float Max31855::get_temperature()
{
	this->spi_cs_pin.set(false);
	wait_us(1); // Must wait for first bit valid

	// Read 32 bits
/*	uint32_t data = spi->write(0);
	data = data<<8;
	data |= spi->write(0);
	data = data<<8;
	data |= spi->write(0);
	data = data<<8;
	data |= spi->write(0);
*/
	uint32_t data = uint32_t(spi->write(0));
	
	this->spi_cs_pin.set(true);
	
	float temperature;

    //Process temp
    if (data & 0x00010000)
        temperature = 1000.f; //Some form of error.
    else
    {
        data = data >> 18;
        temperature = (data & 0x00001FFF) / 4.f;

        if (data & 0x00002000)
        {
            data = ~data;
            temperature = ((data & 0x00001FFF) + 1) / -4.f;
        }
    }
    return temperature;	
}