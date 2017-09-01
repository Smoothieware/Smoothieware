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
#include "StreamOutputPool.h"
#include "utils.h"

#include "max31865.h"

#include "MRI_Hooks.h"

#define MAX31856_CONFIG_REG            0x00
#define MAX31856_RTDMSB_REG            0x01
#define MAX31856_RTDLSB_REG            0x02
#define MAX31856_HFAULTMSB_REG         0x03
#define MAX31856_HFAULTLSB_REG         0x04
#define MAX31856_LFAULTMSB_REG         0x05
#define MAX31856_LFAULTLSB_REG         0x06
#define MAX31856_FAULTSTAT_REG         0x07

#define MAX31856_CONFIG_BIAS           0x80
#define MAX31856_CONFIG_MODEAUTO       0x40
#define MAX31856_CONFIG_MODEOFF        0x00
#define MAX31856_CONFIG_1SHOT          0x20
#define MAX31856_CONFIG_3WIRE          0x10
#define MAX31856_CONFIG_24WIRE         0x00
#define MAX31856_CONFIG_FAULTSTAT      0x02
#define MAX31856_CONFIG_FILT50HZ       0x01
#define MAX31856_CONFIG_FILT60HZ       0x00

#define MAX31865_FAULT_HIGHTHRESH      0x80
#define MAX31865_FAULT_LOWTHRESH       0x40
#define MAX31865_FAULT_REFINLOW        0x20
#define MAX31865_FAULT_REFINHIGH       0x10
#define MAX31865_FAULT_RTDINLOW        0x08
#define MAX31865_FAULT_OVUV            0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

// at 50hz in AUTO mode, MAX31865 returns a new temperature reading (at worst) every 21ms
#define MINIMUM_READ_INTERVAL_MS 25

// MAX31865 supports SPI mode 1 and 3, autodetects based on SCLK state when CS is low
#define SPI_MODE 3

#define rtd_nominal_resistance_checksum CHECKSUM("rtd_nominal_resistance")
#define reference_resistor_checksum CHECKSUM("reference_resistor")
#define use_50hz_filter_checksum CHECKSUM("use_50hz_filter")
#define chip_select_checksum CHECKSUM("chip_select_pin")
#define spi_channel_checksum CHECKSUM("spi_channel")

Max31865::Max31865() :
    spi(nullptr)
{
    this->last_reading_time = 0;
    this->last_temperature = infinityf();
    this->errors_reported = 0;
}

Max31865::~Max31865()
{
    delete spi;
}

void Max31865::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    this->rtd_nominal_resistance = THEKERNEL->config->value(module_checksum, name_checksum, rtd_nominal_resistance_checksum)->by_default(100.0f)->as_number();
    this->reference_resistor = THEKERNEL->config->value(module_checksum, name_checksum, reference_resistor_checksum)->by_default(430.0f)->as_number();
    this->use_50hz_filter = THEKERNEL->config->value(module_checksum, name_checksum, use_50hz_filter_checksum)->by_default(false)->as_bool();

    // Chip select
    this->spi_cs_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, chip_select_checksum)->by_default("0.16")->as_string());
    this->spi_cs_pin.set(true);
    this->spi_cs_pin.as_output();

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

    delete spi;
    spi = new mbed::SPI(mosi, miso, sclk);

    init_rtd();
}

void Max31865::init_rtd()
{
    // enable bias, enable automatic conversions, clear pending faults, set 50/60hz filter
    uint8_t cfg = MAX31856_CONFIG_BIAS | MAX31856_CONFIG_MODEAUTO | MAX31856_CONFIG_FAULTSTAT;
    if (this->use_50hz_filter)
        cfg |= MAX31856_CONFIG_FILT50HZ;

    write_register_8(MAX31856_CONFIG_REG, cfg);
    write_register_8(MAX31856_HFAULTMSB_REG, 0xff);
    write_register_8(MAX31856_HFAULTLSB_REG, 0xff);
    write_register_8(MAX31856_LFAULTMSB_REG, 0x00);
    write_register_8(MAX31856_LFAULTLSB_REG, 0x00);

    // initial bias voltage startup time is 10ms
    // first temperature conversion after enabling AUTO mode takes at most 66ms
    this->last_reading_time = us_ticker_read() + ((10 + 66) * 1000);
}

float Max31865::get_temperature()
{
    return this->last_temperature.load();
}

void Max31865::get_raw()
{
    uint16_t rawData = read_register_16(MAX31856_RTDMSB_REG);
    uint16_t adcValue = rawData >> 1;

    float Rt = adc_value_to_resistance(adcValue);
    float t = resistance_to_temperature(Rt);

    THEKERNEL->streams->printf("Max31865: adc= %d/32767, resistance= %f, temp= %f\n", adcValue, Rt, t);
    THEKERNEL->streams->printf("RTD nominal resistance= %f, reference resistor= %f, using %dHz filter\n", 
        this->rtd_nominal_resistance, this->reference_resistor, this->use_50hz_filter ? 50 : 60);

    // reset in case of fault
    if (rawData & 1)
    {
        print_errors(true);
        init_rtd();
    }
}

// ask the temperature sensor hardware for a value, store it in a buffer
void Max31865::on_idle()
{
    // wait if next temperature conversion is not yet ready
    if ((us_ticker_read() - last_reading_time) < MINIMUM_READ_INTERVAL_MS * 1000)
        return;

    float t = infinityf();

    uint16_t rawData = read_register_16(MAX31856_RTDMSB_REG);
    if (!(rawData & 1))
    {
        uint16_t adcValue = (rawData >> 1);

        float Rt = adc_value_to_resistance(adcValue);
        t = resistance_to_temperature(Rt);
    }
    else
    {
        // reset in case of fault
        print_errors();
        init_rtd();
    }

    this->last_reading_time = us_ticker_read();
    this->last_temperature.store(t);
}

float Max31865::adc_value_to_resistance(uint16_t adcValue)
{
    return (adcValue / 32767.0f) * this->reference_resistor;
}

float Max31865::resistance_to_temperature(float Rt)
{
    float Z1 = -RTD_A;
    float Z2 = RTD_A * RTD_A - (4 * RTD_B);
    float Z3 = (4 * RTD_B) / this->rtd_nominal_resistance;
    float Z4 = 2 * RTD_B;

    float temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;

    return temp;
}

void Max31865::print_errors(bool always)
{
    uint8_t fault = read_register_8(MAX31856_FAULTSTAT_REG);
    uint8_t msgReported = always ? 0 : this->errors_reported;

    if ((fault & 0x80) && !(msgReported & 0x80))
        THEKERNEL->streams->printf("Max31865 error: RTD input is disconnected\n");
    if ((fault & 0x40) && !(msgReported & 0x40))
        THEKERNEL->streams->printf("Max31865 error: RTD input is shorted\n");
    if ((fault & 0x20) && !(msgReported & 0x20))
        THEKERNEL->streams->printf("Max31865 error: VREF- is greater than 0.85 * VBIAS, FORCE- open\n");
    if ((fault & 0x10) && !(msgReported & 0x10))
        THEKERNEL->streams->printf("Max31865 error: VREF- is less than 0.85 * VBIAS, FORCE- open\n");
    if ((fault & 0x08) && !(msgReported & 0x08))
        THEKERNEL->streams->printf("Max31865 error: VRTD- is less than 0.85 * VBIAS, FORCE- open\n");
    if ((fault & 0x04) && !(msgReported & 0x04))
        THEKERNEL->streams->printf("Max31865 error: Overvoltage or undervoltage fault\n");
    if ((fault & 0x03) && !(msgReported & 0x03))
        THEKERNEL->streams->printf("Max31865 error: Unspecified error\n");

    this->errors_reported = fault;
}

uint8_t Max31865::read_register_8(uint8_t reg)
{
    // always set the mode to ensure SCLK is the right polarity before lowering CS
    spi->format(8, SPI_MODE);

    this->spi_cs_pin.set(false);

    spi->write(reg & 0x7f);
    uint8_t val = spi->write(0);

    this->spi_cs_pin.set(true);

    return val;
}

uint16_t Max31865::read_register_16(uint8_t reg)
{
    // always set the mode to ensure SCLK is the right polarity before lowering CS
    spi->format(8, SPI_MODE);

    this->spi_cs_pin.set(false);

    spi->write(reg & 0x7f);
    uint8_t msb = spi->write(0);
    uint8_t lsb = spi->write(0);

    this->spi_cs_pin.set(true);

    return (msb << 8) | lsb;
}

void Max31865::write_register_8(uint8_t reg, uint8_t val)
{
    // always set the SPI mode to ensure SCLK is the right polarity before lowering CS
    spi->format(8, SPI_MODE);

    this->spi_cs_pin.set(false);

    spi->write(reg | 0x80);
    spi->write(val);

    this->spi_cs_pin.set(true);
}
