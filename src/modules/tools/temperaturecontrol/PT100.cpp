#include "PT100.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

#include <fastmath.h>

#define UNDEFINED -1

#define amplifier_pin_checksum CHECKSUM("amplifier_pin")      //adc pin
#define baseline_temp_checksum CHECKSUM("baseline_temp")      // baseline temperature
#define baselineadc_checksum CHECKSUM("baseline_adc_value")   // adc output at baseline temperature
#define adcdeltadeg_checksum CHECKSUM("adc_delta_per_degree") // adc delta per degree

PT100::PT100()
{
    this->min_temp = 400;
    this->max_temp = 0;
}
PT100::~PT100()
{
}
void PT100::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    this->amplifier_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, amplifier_pin_checksum)->required()->as_string());
    this->baseline_temp = THEKERNEL->config->value(module_checksum, name_checksum, baseline_temp_checksum)->by_default(100)->as_number();
    this->baseline_adc_value = THEKERNEL->config->value(module_checksum, name_checksum, baselineadc_checksum)->by_default(4500)->as_number();
    this->adc_delta_deg = THEKERNEL->config->value(module_checksum, name_checksum, adcdeltadeg_checksum)->by_default(41)->as_number();

    THEKERNEL->adc->enable_pin(&amplifier_pin);
}

float PT100::get_temperature()
{
    float t = adc_value_to_temperature(get_adc_reading());
    // keep track of min/max for M305
    if (t > max_temp)
        max_temp = t;
    if (t < min_temp)
        min_temp = t;
    return t;
}

void PT100::get_raw()
{
    int adc_value = get_adc_reading();
    float t = adc_value_to_temperature(adc_value);
    THEKERNEL->streams->printf("PT100: adc= %d, baseTemp= %f, delta/deg=%d, baseAdc=%d, temp= %f\n", 
                                adc_value,this->baseline_temp,this->adc_delta_deg,this->baseline_adc_value, t);
    // reset the min/max
    min_temp = max_temp = t;
}

int PT100::get_adc_reading()
{
    return THEKERNEL->adc->read(&amplifier_pin);
}

float PT100::adc_value_to_temperature(uint32_t adc_value)
{
    //PT100 indicates it has 100Ω resistance at 0C. It is supposed to have a very linear resistnace curve.
    //ADC in Smoothie board is pretty linear with 10bit precision.
    //the formula below uses that facto, and calculates temperature given baseline temp, adc reading and per degree adc slope
    return ((adc_value - this->baseline_adc_value * 1.0) / this->adc_delta_deg) + this->baseline_temp;
}