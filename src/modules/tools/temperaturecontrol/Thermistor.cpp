/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Thermistor.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "Config.h"
#include "checksumm.h"
#include "Adc.h"
#include "ConfigValue.h"
#include "libs/Median.h"
#include "utils.h"
#include "StreamOutputPool.h"

// a const list of predefined thermistors
#include "predefined_thermistors.h"

#include <fastmath.h>

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
#define rt_curve_checksum                  CHECKSUM("rt_curve")
#define coefficients_checksum              CHECKSUM("coefficients")
#define use_beta_table_checksum            CHECKSUM("use_beta_table")


Thermistor::Thermistor()
{
    bad_config = false;
    min_temp = 999;
    max_temp = 0;
    thermistor_number = 0; // not a predefined thermistor
    thermistor.calc_fn = nullptr;
}

Thermistor::~Thermistor()
{
}

// Get configuration from the config file
void Thermistor::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum)
{
    // Values are here : http://reprap.org/wiki/Thermistor
    this->thermistor.r0   = 100000;
    this->thermistor.t0   = 25;
    this->thermistor.r1   = 0;
    this->thermistor.r2   = 4700;
    this->thermistor.beta = 4066;
    this->thermistor.calc_fn = &calc_beta;

    bool found = false;
    int cnt = 0;
    // load a predefined thermistor name if found
    string thr = THEKERNEL->config->value(module_checksum, name_checksum, thermistor_checksum)->by_default("")->as_string();
    if (!thr.empty()) {
	for (auto& i : predefined_thermistors) {
            cnt++;
	    if (thr.compare(i.name) == 0) {
		this->thermistor = i;
		found = true;
           	break;
            }
      	}
        thermistor_number = found ? cnt : 0;
    }

    // Preset values are overriden by specified values
    this->thermistor.beta = THEKERNEL->config->value(module_checksum, name_checksum, beta_checksum)->by_default(thermistor.beta)->as_number(); // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    this->thermistor.r0 = THEKERNEL->config->value(module_checksum, name_checksum, r0_checksum)->by_default(thermistor.r0)->as_number(); // Stated resistance eg. 100K
    this->thermistor.t0 = THEKERNEL->config->value(module_checksum, name_checksum, t0_checksum)->by_default(thermistor.t0)->as_number(); // Temperature at stated resistance, eg. 25C

    this->thermistor.r1 = THEKERNEL->config->value(module_checksum, name_checksum, r1_checksum)->by_default(thermistor.r1)->as_number();
    this->thermistor.r2 = THEKERNEL->config->value(module_checksum, name_checksum, r2_checksum)->by_default(thermistor.r2)->as_number();

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&this->thermistor_pin);

    // specify the three Steinhart-Hart coefficients
    // specified as three comma separated floats, no spaces
    string coef= THEKERNEL->config->value(module_checksum, name_checksum, coefficients_checksum)->by_default("")->as_string();

    // speficy three temp,resistance pairs, best to use 25° 150° 240° and the coefficients will be calculated
    // specified as 25.0,100000.0,150.0,1355.0,240.0,203.0 which is temp in °C,resistance in ohms
    string rtc= THEKERNEL->config->value(module_checksum, name_checksum, rt_curve_checksum)->by_default("")->as_string();
    if(!rtc.empty()) {
        // use the http://en.wikipedia.org/wiki/Steinhart-Hart_equation instead of beta, as it is more accurate over the entire temp range
        // we use three temps/resistor values taken from the thermistor R-C curve found in most datasheets
        // eg http://sensing.honeywell.com/resistance-temperature-conversion-table-no-16, we take the resistance for 25°,150°,240° and the resistance in that table is 100000*R-T Curve coefficient
        // eg http://www.atcsemitec.co.uk/gt-2_thermistors.html for the semitec is even easier as they give the resistance in column 8 of the R/T table

        // then calculate the three Steinhart-Hart coefficients
        // format in config is T1,R1,T2,R2,T3,R3 if all three are not sepcified we revert to an invalid config state
        std::vector<float> trl= parse_number_list(rtc.c_str());
        if(trl.size() != 6) {
            // punt we need 6 numbers, three pairs
            THEKERNEL->streams->printf("Error in config need 6 numbers for Steinhart-Hart\n");
            this->bad_config= true;
            return;
        }

        // calculate the coefficients
        std::tie(this->thermistor.c1, this->thermistor.c2, this->thermistor.c3) = calculate_steinhart_hart_coefficients(trl[0], trl[1], trl[2], trl[3], trl[4], trl[5]);
        this->thermistor.calc_fn = &calc_sh;

    }else if(!coef.empty()) {
        // the three Steinhart-Hart coefficients
        // format in config is C1,C2,C3 if three are not specified we revert to an invalid config state
        std::vector<float> v= parse_number_list(coef.c_str());
        if(v.size() != 3) {
            // punt we need 6 numbers, three pairs
            THEKERNEL->streams->printf("Error in config need 3 Steinhart-Hart coefficients\n");
            this->bad_config= true;
            return;
        }

        this->thermistor.c1 = v[0];
        this->thermistor.c2 = v[1];
        this->thermistor.c3 = v[2];
        this->thermistor.calc_fn = &calc_sh;

    } else if (!found) {
        THEKERNEL->streams->printf("Error in config need rt_curve, coefficients, beta or a valid predefined thermistor defined\n");
        this->bad_config= true;
        return;
    }

}

// print out predefined thermistors
void Thermistor::print_predefined_thermistors(StreamOutput* s)
{
    int cnt= 1;
    s->printf("S/H table\n");
    for (auto& i : predefined_thermistors) {
        s->printf("%d - %s\n", cnt++, i.name);
    }
}

// calculate the coefficients from the supplied three Temp/Resistance pairs
// copied from https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
std::tuple<float,float,float> Thermistor::calculate_steinhart_hart_coefficients(float t1, float r1, float t2, float r2, float t3, float r3)
{
    float l1 = logf(r1);
    float l2 = logf(r2);
    float l3 = logf(r3);

    float y1 = 1.0F / (t1 + 273.15F);
    float y2 = 1.0F / (t2 + 273.15F);
    float y3 = 1.0F / (t3 + 273.15F);
    float x = (y2 - y1) / (l2 - l1);
    float y = (y3 - y1) / (l3 - l1);
    float c = (y - x) / ((l3 - l2) * (l1 + l2 + l3));
    float b = x - c * (powf(l1,2) + powf(l2,2) + l1 * l2);
    float a = y1 - (b + powf(l1,2) * c) * l1;

    if(c < 0) {
        THEKERNEL->streams->printf("WARNING: negative coefficient in calculate_steinhart_hart_coefficients. Something may be wrong with the measurements\n");
        c = -c;
    }
    return std::make_tuple(a, b, c);
}

float Thermistor::get_temperature()
{
    if(bad_config) return infinityf();
    float t= adc_value_to_temperature(new_thermistor_reading());
    // keep track of min/max for M305
    if(t > max_temp) max_temp= t;
    if(t < min_temp) min_temp= t;
    return t;
}

void Thermistor::get_raw()
{
    if(this->bad_config) {
       THEKERNEL->streams->printf("WARNING: The config is bad for this temperature sensor\n");
    }

    int adc_value= new_thermistor_reading();
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();

     // resistance of the thermistor in ohms
    float r = this->thermistor.r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (this->thermistor.r1 > 0.0F)
    	r = (this->thermistor.r1 * r) / (this->thermistor.r1 - r);

    THEKERNEL->streams->printf("adc= %d, resistance= %f\n", adc_value, r);

    float t = thermistor.calc_fn ? thermistor.calc_fn(&this->thermistor, r) : 0;

    THEKERNEL->streams->printf("temp= %f, min= %f, max= %f, delta= %f\n", t, min_temp, max_temp, max_temp - min_temp);
    // if using a predefined thermistor show its name and which table it is from
    if (thermistor_number != 0) {
        string name = predefined_thermistors[this->thermistor_number - 1].name;
        THEKERNEL->streams->printf("Using predefined thermistor %d: %s\n", thermistor_number & 0x7F, name.c_str());
    }

    // reset the min/max
    min_temp= max_temp= t;
}

float Thermistor::adc_value_to_temperature(uint32_t adc_value)
{
    const uint32_t max_adc_value= THEKERNEL->adc->get_max_value();
    if ((adc_value >= max_adc_value) || (adc_value == 0))
        return infinityf();

    // resistance of the thermistor in ohms
    float r = this->thermistor.r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (this->thermistor.r1 > 0.0F)
    	r = (this->thermistor.r1 * r) / (this->thermistor.r1 - r);

    if (r > 800000/*thermistor.r0 * 8*/)
    	return infinityf(); // 800k is probably open circuit

    return thermistor.calc_fn ? thermistor.calc_fn(&this->thermistor, r) : infinityf();
}

int Thermistor::new_thermistor_reading()
{
    // filtering now done in ADC
    return THEKERNEL->adc->read(&thermistor_pin);
}

bool Thermistor::set_optional(const sensor_options_t& options) {
    bool define_beta= false;
    bool change_beta= false;
    uint8_t define_shh= 0;
    uint8_t predefined= 0;

    for (auto &i : options) {
        switch (i.first) {
            case 'B':
            	this->thermistor.beta = i.second;
            	define_beta = true;
            	break;
            case 'R':
            	this->thermistor.r0 = i.second;
            	change_beta = true;
            	break;
            case 'X':
            	this->thermistor.t0 = i.second;
            	change_beta = true;
            	break;
            case 'I':
            	this->thermistor.c1 = i.second;
            	define_shh++;
            	break;
            case 'J':
            	this->thermistor.c2 = i.second;
            	define_shh++;
            	break;
            case 'K':
            	this->thermistor.c3 = i.second;
            	define_shh++;
            	break;
            case 'P':
            	predefined = roundf(i.second);
            	break;
        }
    }

    if(predefined != 0) {
        if(define_beta || change_beta || define_shh != 0) {
            // cannot use a predefined with any other option
            this->bad_config= true;
            return false;
        }

            // use the predefined S/H table
		uint8_t n = predefined - 1;
     	if (n >= sizeof(predefined_thermistors) / sizeof(thermistor_t)) {
                // not a valid index
                return false;
            }
    	auto &i = predefined_thermistors[n];
    	this->thermistor = i;
    	thermistor_number = predefined;
     	bad_config = false;
            return true;
        }

	bool error = false;
    if (error) {
        bad_config = true;
        return false;
    }

    if(this->bad_config) this->bad_config= false;

    return true;
}

bool Thermistor::get_optional(sensor_options_t& options) {
    if(thermistor_number != 0) {
        options['P']= thermistor_number;
        return true;
    }

    return true;
};
