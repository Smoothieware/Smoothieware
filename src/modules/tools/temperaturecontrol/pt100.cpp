#include "pt100.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"

#include <fastmath.h>

#define UNDEFINED -1

#define pt100linear_checksum               CHECKSUM("pt100linear")        // if 1 then use slope/yintercept, if 0 use
                                                                          // A, B, R0
#define thermistor_pin_checksum            CHECKSUM("thermistor_pin")
#define ampmod1_pin_checksum               CHECKSUM("ampmod1_pin")        // UP! printer uses this to "energize" the RTD
#define ampmod2_pin_checksum               CHECKSUM("ampmod2_pin")        // set as nc if you don't need to energize RTD
#define slope_checksum                     CHECKSUM("slope")              // We are looking RTD as "linear" so tune your
#define yintercept_checksum                CHECKSUM("yintercept")         // SLOPE and Y-Intercept to have minimal error
                                                                          // in the region you use 

#ifndef NO_RTD_CURVE
#define pt100A_checksum                    CHECKSUM("pt100_a")            // A value
#define pt100B_checksum                    CHECKSUM("pt100_b")            // B value
#define pt100R0_checksum                   CHECKSUM("pt100_r0")           // R0 value
#define pt100amptype_checksum              CHECKSUM("pt100_amptype")      // Amp type between RTD and ADC input
#endif                                                                    // attm only supported ones are 
                                                                          // 0: e3d pt100 amp http://wiki.e3d-online.com/wiki/E3D_PT100_Amplifier_Documentation

PT100::PT100()
{
    this->bad_config = false;
    this->use_ampmod1= false;
    this->use_ampmod2= false;
    this->min_temp= 999;
    this->max_temp= 0;
    this->m = 0;             //use as slope or A 
    this->b = 0;             //use as yintercept or B
#ifndef NO_RTD_CURVE
    this->r0= 0;             // R0 - temp of the RTD and 0C (100 for PT100)
    this->use_linear= true;  // true: use linear, false: use fancy math
#endif
}

PT100::~PT100()
{
}

void PT100::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    this->ampmod1_pin.from_string(   THEKERNEL->config->value(module_checksum, name_checksum, ampmod1_pin_checksum    )->by_default("nc")->as_string());
    this->ampmod2_pin.from_string(   THEKERNEL->config->value(module_checksum, name_checksum, ampmod2_pin_checksum    )->by_default("nc")->as_string());
#ifndef NO_RTD_CURVE
    this->use_linear = THEKERNEL->config->value(module_checksum, name_checksum,  pt100linear_checksum)->by_default(1)->as_number() > 0;

    if (this->use_linear){
#endif
      this->m = THEKERNEL->config->value(module_checksum, name_checksum, slope_checksum      )->by_default(  0.0257604875F )->as_number(); // default value for UP! 
      this->b = THEKERNEL->config->value(module_checksum, name_checksum, yintercept_checksum )->by_default(-18.54F         )->as_number(); // PT100 in the hotend
#ifndef NO_RTD_CURVE
    }else{
      this->m = THEKERNEL->config->value(module_checksum, name_checksum, pt100A_checksum )->by_default( 3.9083E-3F )->as_number(); // ALFA (for PT100 ITS-90 it is 3.9083E-3 )
      this->b = THEKERNEL->config->value(module_checksum, name_checksum, pt100B_checksum )->by_default( 5.775E-7F  )->as_number(); // BETA for the RTD (for PT100 ITS-90 it is 5.775E-7 )
      this->r0= THEKERNEL->config->value(module_checksum, name_checksum, pt100R0_checksum)->by_default( 100        )->as_number(); // R0 - resistance on 0C (for PT100 it is 100)
      this->amptype= THEKERNEL->config->value(module_checksum, name_checksum, pt100amptype_checksum)->by_default(0 )->as_number(); // Amp type
    }
#endif
    THEKERNEL->adc->enable_pin(&thermistor_pin);

    if(this->ampmod1_pin.connected()){
        this->ampmod1_pin.as_output();
        this->ampmod1_pin.set(true);
        this->use_ampmod1= true;
    }

    if(this->ampmod2_pin.connected()){
        this->ampmod2_pin.as_output();
        this->ampmod2_pin.set(true);
        this->use_ampmod2= true;
    }

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

void PT100::get_raw() //no clue what/where/when is this used for
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

#ifndef NO_RTD_CURVE
    if (this->use_linear){
        // We are using a very short range of temperatures for 3D printing
        // so looking at 150-350C span any RTD is almost linear hence we
        // use the simple linear equation allowing slope and y-intersect to 
        // be configurable so user can tune for particular RTD and tune for 
        // minimal error on the required span
#endif
        t = adc_value * this->m + this->b;
#ifndef NO_RTD_CURVE
    } else {
        // proper RTD calculation should be like this
        // but depending on how the RTD is connected, is there 
        // some amp after it, or any other circuit.. 
        // I'm not using it
        //
        //float R0 = 100;     //R0 - temp of the RTD at zero
        //float res;  //resistance of the RTD that we want temp from
        //float alpha = 3.9083E-3; // A for the RTD (for PT100 ITS-90 it is 3.9083E-3 )
        //float betha = 5.775E-7;  // B for the RTD (for PT100 ITS-90 it is 5.775E-7 )
        //t = (-R0 * alpha + sqrtf(R0 * R0 * + alpha * alpha - 4 * R0 * betha * (R0 - res))) / (2 * R0 * - betha);

        // there is a problem on how to get RES from ADC, especially if there is some AMP in front of ADC pin
        // and usually there is one. I'm solving this by "hardcoding amp type parameters in code"
        // as I can't think how to do it smarter
        switch (this->amptype){
	case 0: // e3d pt100 amp
           t =  805200 * adc_value / (45409000 - 183 * adc_value); // Calculate resistance from ADC value
           break;
        default: // TODO: add more amp types
           t =  805200 * adc_value / (45409000 - 183 * adc_value); 
        }
        t = (-1 * this->r0 * this->m + sqrtf(this->r0 * this->r0 * + this->m * this->m - 4 * this->r0 * this->b * (this->r0 - t ))) / (2 * this->r0 * this->b);
    }
#endif
    return t;
}

int PT100::new_thermistor_reading()
{
    int t;

    if(this->use_ampmod1) this->ampmod1_pin.set(true);
    if(this->use_ampmod2) this->ampmod2_pin.set(true);

    // filtering now done in ADC
    t = THEKERNEL->adc->read(&thermistor_pin);

    if(this->use_ampmod1) this->ampmod1_pin.set(false);
    if(this->use_ampmod2) this->ampmod2_pin.set(false);

    return t;
}


