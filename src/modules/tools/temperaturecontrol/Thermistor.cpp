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


Thermistor::Thermistor()
{
    this->bad_config = false;
    this->use_steinhart_hart= false;
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

    // specify the three Steinhart-Hart coefficients
    string coef= THEKERNEL->config->value(module_checksum, name_checksum, coefficients_checksum)->by_default("")->as_string();

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
        calculate_steinhart_hart_coefficients(trl[0], trl[1], trl[2], trl[3], trl[4], trl[5]);

        this->use_steinhart_hart= true;

    }else if(!coef.empty()) {
        std::vector<float> v= parse_number_list(coef.c_str());
        if(v.size() != 3) {
            // punt we need 6 numbers, three pairs
            THEKERNEL->streams->printf("Error in config need 3 Steinhart-Hart coefficients\n");
            this->bad_config= true;
            return;
        }

        this->c1= v[0];
        this->c2= v[1];
        this->c3= v[2];
        this->use_steinhart_hart= true;

    }else {
        // if using beta
        calc_jk();
        this->use_steinhart_hart= false;
    }

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&thermistor_pin);
}

// calculate the coefficients from the supplied three Temp/Resistance pairs
// copied from https://github.com/MarlinFirmware/Marlin/blob/Development/Marlin/scripts/createTemperatureLookupMarlin.py
void Thermistor::calculate_steinhart_hart_coefficients(float t1, float r1, float t2, float r2, float t3, float r3)
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

    this->c1= a;
    this->c2= b;
    this->c3= c;
}

void Thermistor::calc_jk()
{
    // Thermistor math
    j = (1.0F / beta);
    k = (1.0F / (t0 + 273.15F));
}

float Thermistor::get_temperature()
{
    if(bad_config) return infinityf();
    return adc_value_to_temperature(new_thermistor_reading());
}

void Thermistor::get_raw()
{
    int adc_value= new_thermistor_reading();
     // resistance of the thermistor in ohms
    float r = r2 / ((4095.0F / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    THEKERNEL->streams->printf("adc= %d, resistance= %f\n", adc_value, r);

    if(this->use_steinhart_hart) {
        THEKERNEL->streams->printf("S/H c1= %1.10f, c2= %1.10f, c3= %1.10f\n", c1, c2, c3);
        float l = logf(r);
        float t= (1.0F / (this->c1 + this->c2 * l + this->c3 * powf(l,3))) - 273.15F;
        THEKERNEL->streams->printf("S/H temp= %f\n", t);
    }else{
        float t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
        THEKERNEL->streams->printf("beta temp= %f\n", t);
    }
}

float Thermistor::adc_value_to_temperature(int adc_value)
{
    if ((adc_value == 4095) || (adc_value == 0))
        return infinityf();

    // resistance of the thermistor in ohms
    float r = r2 / ((4095.0F / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    float t;
    if(this->use_steinhart_hart) {
        float l = logf(r);
        t= (1.0F / (this->c1 + this->c2 * l + this->c3 * powf(l,3))) - 273.15F;
    }else{
        // use Beta value
        t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
    }

    return t;
}

int Thermistor::new_thermistor_reading()
{
    int last_raw = THEKERNEL->adc->read(&thermistor_pin);
    if (queue.size() >= queue.capacity()) {
        uint16_t l;
        queue.pop_front(l);
    }
    uint16_t r = last_raw;
    queue.push_back(r);
    uint16_t median_buffer[queue.size()];
    for (int i=0; i<queue.size(); i++)
      median_buffer[i] = *queue.get_ref(i);
    uint16_t m = median_buffer[quick_median(median_buffer, queue.size())];
    return m;
}

bool Thermistor::set_optional(const sensor_options_t& options) {
    bool define_beta= false;
    sensor_options_t::const_iterator i;

    // B is beta
    i= options.find('B');
    if(i != options.end()) {
        this->beta= i->second;
        define_beta= true;
    }
    i= options.find('R');
    if(i != options.end()) {
        this->r0= i->second;
        define_beta= true;
    }
    i= options.find('X');
    if(i != options.end()) {
        this->t0= i->second;
        define_beta= true;
    }

    if(!define_beta) {
        // I,J,K ar3 the C1, C2, C3 Steinhart-Hart coefficients
        i= options.find('I');
        if(i != options.end()) {
            this->c1= i->second;
            use_steinhart_hart= true;
        }
        i= options.find('J');
        if(i != options.end()) {
            this->c2= i->second;
            use_steinhart_hart= true;
        }
        i= options.find('K');
        if(i != options.end()) {
            this->c3= i->second;
            use_steinhart_hart= true;
        }

    }else {
        calc_jk();
        use_steinhart_hart= false;
    }

    return true;
}

bool Thermistor::get_optional(sensor_options_t& options) {
    if(use_steinhart_hart) {
        options['I']= this->c1;
        options['J']= this->c2;
        options['K']= this->c3;

    }else{
        options['B']= this->beta;
        options['X']= this->t0;
        options['R']= this->r0;
    }

    return true;
};
