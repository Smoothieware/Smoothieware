#include "pt100.h"
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

#define thermistor_pin_checksum            CHECKSUM("thermistor_pin")
#define ampmod1_pin_checksum               CHECKSUM("ampmod1_pin")
#define ampmod2_pin_checksum               CHECKSUM("ampmod2_pin")


PT100::PT100()
{
    this->bad_config = false;
    min_temp= 999;
    max_temp= 0;
}

PT100::~PT100()
{
}

void PT100::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    // PT100 pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    this->ampmod1_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, ampmod1_pin_checksum )->required()->as_string());
    this->ampmod2_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, ampmod2_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&thermistor_pin);
    this->ampmod1_pin.as_output();
    this->ampmod1_pin.set(true);
    this->ampmod2_pin.as_output();
    this->ampmod2_pin.set(true);

}

float PT100::get_temperature()
{
    if(bad_config) return infinityf();
    float t= adc_value_to_temperature(new_thermistor_reading());
    // keep track of min/max for M305
    if(t > max_temp) max_temp = t;
    if(t < min_temp) min_temp = t;
    return t;
}

void PT100::get_raw()
{
    if(this->bad_config) {
       THEKERNEL->streams->printf("WARNING: The config is bad for this temperature sensor\n");
    }
    int adc_value= new_thermistor_reading();
    float t = adc_value_to_temperature(adc_value);
    THEKERNEL->streams->printf("PT100: adc= %d, temp= %f\n", adc_value, t);
    min_temp= max_temp= t;
}

float PT100::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
    if ((adc_value >= max_adc_value) || (adc_value == 0))
        return infinityf();

    float t;
    t = adc_value / 10.0;
    return t;
}

int PT100::new_thermistor_reading()
{
    int t;

    this->ampmod1_pin.set(true);
    this->ampmod2_pin.set(true);

    // filtering now done in ADC
    t = THEKERNEL->adc->read(&thermistor_pin);

    //this->ampmod1_pin.set(false);
    //this->ampmod2_pin.set(false);
    return t;
}


