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
    this->bad_config = false;
    this->use_steinhart_hart= false;
    this->beta= 0.0F; // not used by default
    min_temp= 999;
    max_temp= 0;
    this->thermistor_number= 0; // not a predefined thermistor
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
    this->r1   = 0;
    this->r2   = 4700;
    this->beta = 4066;

    // force use of beta perdefined thermistor table based on betas
    bool use_beta_table= THEKERNEL->config->value(module_checksum, name_checksum, use_beta_table_checksum)->by_default(false)->as_bool();

    bool found= false;
    int cnt= 0;
    // load a predefined thermistor name if found
    string thermistor = THEKERNEL->config->value(module_checksum, name_checksum, thermistor_checksum)->by_default("")->as_string();
    if(!thermistor.empty()) {
        if(!use_beta_table) {
            for (auto& i : predefined_thermistors) {
                cnt++;
                if(thermistor.compare(i.name) == 0) {
                    this->c1 = i.c1;
                    this->c2 = i.c2;
                    this->c3 = i.c3;
                    this->r1 = i.r1;
                    this->r2 = i.r2;
                    use_steinhart_hart= true;
                    found= true;
                    break;
                }
            }
        }

        // fall back to the old beta pre-defined table if not found above
        if(!found) {
            cnt= 0;
            for (auto& i : predefined_thermistors_beta) {
                cnt++;
                if(thermistor.compare(i.name) == 0) {
                    this->beta = i.beta;
                    this->r0 = i.r0;
                    this->t0 = i.t0;
                    this->r1 = i.r1;
                    this->r2 = i.r2;
                    use_steinhart_hart= false;
                    cnt |= 0x80; // set MSB to indicate beta table
                    found= true;
                    break;
                }
            }
        }

        if(!found) {
            thermistor_number= 0;
        }else{
            thermistor_number= cnt;
        }
    }

    // Preset values are overriden by specified values
    if(!use_steinhart_hart) {
        this->beta = THEKERNEL->config->value(module_checksum, name_checksum, beta_checksum)->by_default(this->beta)->as_number(); // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    }
    this->r0 = THEKERNEL->config->value(module_checksum, name_checksum, r0_checksum  )->by_default(this->r0  )->as_number(); // Stated resistance eg. 100K
    this->t0 = THEKERNEL->config->value(module_checksum, name_checksum, t0_checksum  )->by_default(this->t0  )->as_number(); // Temperature at stated resistance, eg. 25C
    this->r1 = THEKERNEL->config->value(module_checksum, name_checksum, r1_checksum  )->by_default(this->r1  )->as_number();
    this->r2 = THEKERNEL->config->value(module_checksum, name_checksum, r2_checksum  )->by_default(this->r2  )->as_number();

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(module_checksum, name_checksum, thermistor_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&thermistor_pin);

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
        std::tie(this->c1, this->c2, this->c3) = calculate_steinhart_hart_coefficients(trl[0], trl[1], trl[2], trl[3], trl[4], trl[5]);

        this->use_steinhart_hart= true;

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

        this->c1= v[0];
        this->c2= v[1];
        this->c3= v[2];
        this->use_steinhart_hart= true;

    }else if(!use_steinhart_hart) {
        // if using beta
        calc_jk();

    }else if(!found) {
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

    cnt= 129;
    s->printf("Beta table\n");
    for (auto& i : predefined_thermistors_beta) {
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

void Thermistor::calc_jk()
{
    // Thermistor math
    if(beta > 0.0F) {
        j = (1.0F / beta);
        k = (1.0F / (t0 + 273.15F));
    }else{
        THEKERNEL->streams->printf("WARNING: beta cannot be 0\n");
        this->bad_config= true;
    }
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
    float r = r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    THEKERNEL->streams->printf("adc= %d, resistance= %f\n", adc_value, r);

    float t;
    if(this->use_steinhart_hart) {
        THEKERNEL->streams->printf("S/H c1= %1.18f, c2= %1.18f, c3= %1.18f\n", c1, c2, c3);
        float l = logf(r);
        t= (1.0F / (this->c1 + this->c2 * l + this->c3 * powf(l,3))) - 273.15F;
        THEKERNEL->streams->printf("S/H temp= %f, min= %f, max= %f, delta= %f\n", t, min_temp, max_temp, max_temp-min_temp);
    }else{
        t= (1.0F / (k + (j * logf(r / r0)))) - 273.15F;
        THEKERNEL->streams->printf("beta temp= %f, min= %f, max= %f, delta= %f\n", t, min_temp, max_temp, max_temp-min_temp);
    }

    // if using a predefined thermistor show its name and which table it is from
    if(thermistor_number != 0) {
        string name= (thermistor_number&0x80) ? predefined_thermistors_beta[(thermistor_number&0x7F)-1].name :  predefined_thermistors[thermistor_number-1].name;
        THEKERNEL->streams->printf("Using predefined thermistor %d in %s table: %s\n", thermistor_number&0x7F, (thermistor_number&0x80)?"Beta":"S/H", name.c_str());
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
    float r = r2 / (((float)max_adc_value / adc_value) - 1.0F);
    if (r1 > 0.0F) r = (r1 * r) / (r1 - r);

    if(r > this->r0 * 8) return infinityf(); // 800k is probably open circuit

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
    // filtering now done in ADC
    return THEKERNEL->adc->read(&thermistor_pin);
}

bool Thermistor::set_optional(const sensor_options_t& options) {
    bool define_beta= false;
    bool change_beta= false;
    uint8_t define_shh= 0;
    uint8_t predefined= 0;

    for(auto &i : options) {
        switch(i.first) {
            case 'B': this->beta= i.second; define_beta= true; break;
            case 'R': this->r0= i.second; change_beta= true; break;
            case 'X': this->t0= i.second; change_beta= true; break;
            case 'I': this->c1= i.second; define_shh++; break;
            case 'J': this->c2= i.second; define_shh++; break;
            case 'K': this->c3= i.second; define_shh++; break;
            case 'P': predefined= roundf(i.second); break;
        }
    }

    if(predefined != 0) {
        if(define_beta || change_beta || define_shh != 0) {
            // cannot use a predefined with any other option
            this->bad_config= true;
            return false;
        }

        if(predefined & 0x80) {
            // use the predefined beta table
            uint8_t n= (predefined&0x7F)-1;
            if(n >= sizeof(predefined_thermistors_beta) / sizeof(thermistor_beta_table_t)) {
                // not a valid index
                return false;
            }
            auto &i= predefined_thermistors_beta[n];
            this->beta = i.beta;
            this->r0 = i.r0;
            this->t0 = i.t0;
            this->r1 = i.r1;
            this->r2 = i.r2;
            use_steinhart_hart= false;
            calc_jk();
            thermistor_number= predefined;
            this->bad_config= false;
            return true;

        }else {
            // use the predefined S/H table
            uint8_t n= predefined-1;
            if(n >= sizeof(predefined_thermistors) / sizeof(thermistor_table_t)) {
                // not a valid index
                return false;
            }
            auto &i= predefined_thermistors[n];
            this->c1 = i.c1;
            this->c2 = i.c2;
            this->c3 = i.c3;
            this->r1 = i.r1;
            this->r2 = i.r2;
            use_steinhart_hart= true;
            thermistor_number= predefined;
            this->bad_config= false;
            return true;
        }
    }

    bool error= false;
    // if in Steinhart-Hart mode make sure B is specified, if in beta mode make sure all C1,C2,C3 are set and no beta settings
    // this is needed if swapping between modes
    if(use_steinhart_hart && define_shh == 0 && !define_beta) error= true; // if switching from SHH to beta need to specify new beta
    if(!use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if in beta mode and switching to SHH malke sure no beta settings are set
    if(!use_steinhart_hart && !(define_beta || change_beta) && define_shh != 3) error= true; // if in beta mode and switching to SHH must specify all three SHH
    if(use_steinhart_hart && define_shh > 0 && (define_beta || change_beta)) error= true; // if setting SHH anfd already in SHH do not specify any beta values

    if(error) {
        this->bad_config= true;
        return false;
    }
    if(define_beta || change_beta) {
        calc_jk();
        use_steinhart_hart= false;
    }else if(define_shh > 0) {
        use_steinhart_hart= true;
    }else{
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
