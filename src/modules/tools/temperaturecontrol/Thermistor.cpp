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
#include "Adc.h"
#include "ConfigValue.h"
#include "libs/Median.h"
#include "Thermistor.h"
#include "SlowTicker.h"

// a const list of predefined thermistors
#include "predefined_thermistors.h"

#include "MRI_Hooks.h"

#define UNDEFINED -1

#define thermistor_checksum                CHECKSUM("thermistor")
#define r0_checksum                        CHECKSUM("r0")
#define t0_checksum                        CHECKSUM("t0")
#define beta_checksum                      CHECKSUM("beta")
#define vadc_checksum                      CHECKSUM("vadc")
#define vcc_checksum                       CHECKSUM("vcc")
#define r1_checksum                        CHECKSUM("r1")
#define r2_checksum                        CHECKSUM("r2")
#define thermistor_pin_checksum            CHECKSUM("thermistor_pin")
#define oversample_freq_checksum           CHECKSUM("oversample_freq")

Thermistor::Thermistor()
{
}

Thermistor::~Thermistor()
{
}

// Get configuration from the config file
void Thermistor::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    // Values are here : http://reprap.org/wiki/Thermistor
    this->r0   = 100000;
    this->t0   = 25;
    this->beta = 4066;
    this->r1   = 0;
    this->r2   = 4700;

    // load a predefined thermistor name if found
    string thermistor = THEKERNEL->config->value(module_checksum, name_checksum, thermistor_checksum)->as_string();
    for (auto i : predefined_thermistors) {
        if(thermistor.compare(i.name) == 0) {
            this->beta = i.beta;
            this->r0   = i.r0;
            this->t0   = i.t0;
            this->r1   = i.r1;
            this->r2   = i.r2;
            break;
        }
    }

    // Preset values are overriden by specified values
    this->r0 = THEKERNEL->config->value(module_checksum, name_checksum, r0_checksum  )->by_default(this->r0  )->as_number(); // Stated resistance eg. 100K
    this->t0 = THEKERNEL->config->value(module_checksum, name_checksum, t0_checksum  )->by_default(this->t0  )->as_number(); // Temperature at stated resistance, eg. 25C
    this->beta = THEKERNEL->config->value(module_checksum, name_checksum, beta_checksum)->by_default(this->beta)->as_number(); // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    this->r1 = THEKERNEL->config->value(module_checksum, name_checksum, r1_checksum  )->by_default(this->r1  )->as_number();
    this->r2 = THEKERNEL->config->value(module_checksum, name_checksum, r2_checksum  )->by_default(this->r2  )->as_number();
    this->oversample_freq = THEKERNEL->config->value(module_checksum, name_checksum, oversample_freq_checksum)->by_default(0)->as_int();

    // Thermistor math
    j = (1.0 / beta);
    k = (1.0 / (t0 + 273.15));

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&thermistor_pin);
    
    if (this->oversample_freq > 0)
    {
        this->oversample_count = 1;
        this->oversample_sum = 0;
        THEKERNEL->slow_ticker->attach(this->oversample_freq, this, &Thermistor::oversample_tick);
    }
}

float Thermistor::get_temperature()
{
    return adc_value_to_temperature(new_thermistor_reading());
}

float Thermistor::adc_value_to_temperature(int adc_value)
{
    if ((adc_value >= 65530) || (adc_value <= 10))
        return infinityf();
    float r = r2 / ((4095.0f * 16.0f / adc_value) - 1.0f);
    if (r1 > 0)
        r = (r1 * r) / (r1 - r);
    return (1.0f / (k + (j * logf(r / r0)))) - 273.15f;
}

int Thermistor::new_thermistor_reading()
{
    int last_raw;

    // Data is always scaled by 16, because that is the most we can fit
    // in the uint16_t median buffer.
    if (oversample_freq == 0)
    {
        last_raw = 16 * THEKERNEL->adc->read(&thermistor_pin);
    }
    else
    {
        last_raw = 16 * oversample_sum / oversample_count;
        oversample_sum = THEKERNEL->adc->read(&thermistor_pin);
        oversample_count = 1;
    }

    if (queue.size() >= queue.capacity()) {
        uint16_t l;
        queue.pop_front(l);
    }
    uint16_t r = last_raw;
    queue.push_back(r);

    for (int i=0; i<queue.size(); i++)
      median_buffer[i] = *queue.get_ref(i);
    uint16_t m = median_buffer[quick_median(median_buffer, queue.size())];
    return m;
}

uint32_t Thermistor::oversample_tick(uint32_t dummy)
{
    int value = THEKERNEL->adc->read(&thermistor_pin);

    // LPC1769 ADC seems to give spurious spikes sometimes when the stepper
    // drivers are active. Filter most of them out before oversampling. The
    // median filter will take out the rest.
    int avg = oversample_sum / oversample_count;
    if (value > avg - 5 && value < avg + 5)
    {
        oversample_count++;
        oversample_sum += value;
    }

    return 0;
}
