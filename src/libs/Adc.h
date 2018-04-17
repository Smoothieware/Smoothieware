/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef ADC_H
#define ADC_H

#include "PinNames.h" // mbed.h lib

#include <cmath>

class Pin;
namespace mbed {
    class ADC;
}

// define how many bits of extra resolution required
// 2 bits means the 12bit ADC is 14 bits of resolution
#define OVERSAMPLE 2

class Adc
{
public:
    Adc();
    void enable_pin(Pin *pin);
    unsigned int read(Pin *pin);

    static Adc *instance;
    void new_sample(int chan, uint32_t value);
    // return the maximum ADC value, base is 12bits 4095.
#ifdef OVERSAMPLE
    int get_max_value() const { return 4095 << OVERSAMPLE;}
#else
    int get_max_value() const { return 4095;}
#endif

private:
    PinName _pin_to_pinname(Pin *pin);
    mbed::ADC *adc;

    static const int num_channels= 6;
#ifdef OVERSAMPLE
    // we need 4^n sample to oversample and we get double that to filter out spikes
    static const int num_samples= powf(4, OVERSAMPLE)*2;
#else
    static const int num_samples= 8;
#endif
    // buffers storing the last num_samples readings for each channel
    uint16_t sample_buffers[num_channels][num_samples];
};

#endif
