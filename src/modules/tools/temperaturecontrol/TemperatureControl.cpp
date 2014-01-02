/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

// TODO : THIS FILE IS LAME, MUST BE MADE MUCH BETTER

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "TemperatureControl.h"
#include "TemperatureControlPool.h"
#include "libs/Pin.h"
#include "libs/Median.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"

#include "MRI_Hooks.h"

TemperatureControl::TemperatureControl(uint16_t name) :
  name_checksum(name), waiting(false), min_temp_violated(false) {}

void TemperatureControl::on_module_loaded(){

    // We start not desiring any temp
    this->target_temperature = UNDEFINED;

    // Settings
    this->on_config_reload(this);

    this->acceleration_factor = 10;

    // Register for events
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

void TemperatureControl::on_main_loop(void* argument){
    if (this->min_temp_violated) {
        THEKERNEL->streams->printf("Error: MINTEMP triggered on P%d.%d! check your thermistors!\n", this->thermistor_pin.port_number, this->thermistor_pin.pin);
        this->min_temp_violated = false;
    }
}

// Get configuration from the config file
void TemperatureControl::on_config_reload(void* argument){

    // General config
    this->set_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, set_m_code_checksum)->by_default(104)->as_number();
    this->set_and_wait_m_code = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, set_and_wait_m_code_checksum)->by_default(109)->as_number();
    this->get_m_code          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();
    this->readings_per_second = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(20)->as_number();

    this->designator          = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Values are here : http://reprap.org/wiki/Thermistor
    this->r0   = 100000;
    this->t0   = 25;
    this->beta = 4066;
    this->r1   = 0;
    this->r2   = 4700;

    // Preset values for various common types of thermistors
    ConfigValue* thermistor = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, thermistor_checksum);
    if(       thermistor->value.compare("EPCOS100K"    ) == 0 ){ // Default
    }else if( thermistor->value.compare("RRRF100K"     ) == 0 ){ this->beta = 3960;
    }else if( thermistor->value.compare("RRRF10K"      ) == 0 ){ this->beta = 3964; this->r0 = 10000; this->r1 = 680; this->r2 = 1600;
    }else if( thermistor->value.compare("Honeywell100K") == 0 ){ this->beta = 3974;
    }else if( thermistor->value.compare("Semitec"      ) == 0 ){ this->beta = 4267;
    }else if( thermistor->value.compare("HT100K"       ) == 0 ){ this->beta = 3990; }

    // Preset values are overriden by specified values
    this->r0 =                  THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, r0_checksum  )->by_default(this->r0  )->as_number();               // Stated resistance eg. 100K
    this->t0 =                  THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, t0_checksum  )->by_default(this->t0  )->as_number();               // Temperature at stated resistance, eg. 25C
    this->beta =                THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, beta_checksum)->by_default(this->beta)->as_number();               // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    this->r1 =                  THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, r1_checksum  )->by_default(this->r1  )->as_number();
    this->r2 =                  THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, r2_checksum  )->by_default(this->r2  )->as_number();

    this->preset1 =             THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, preset1_checksum)->by_default(0)->as_number();
    this->preset2 =             THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, preset2_checksum)->by_default(0)->as_number();


    // Thermistor math
    j = (1.0 / beta);
    k = (1.0 / (t0 + 273.15));

    // sigma-delta output modulation
    o = 0;

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, thermistor_pin_checksum )->required()->as_string());
    THEKERNEL->adc->enable_pin(&thermistor_pin);

    // Heater pin
    this->heater_pin.from_string(    THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, heater_pin_checksum)->required()->as_string())->as_output();
    this->heater_pin.max_pwm(        THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, max_pwm_checksum)->by_default(255)->as_number() );
    this->heater_pin.set(0);

    set_low_on_debug(heater_pin.port_number, heater_pin.pin);

    // activate SD-DAC timer
    THEKERNEL->slow_ticker->attach( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, pwm_frequency_checksum)->by_default(2000)->as_number() , &heater_pin, &Pwm::on_tick);

    // reading tick
    THEKERNEL->slow_ticker->attach( this->readings_per_second, this, &TemperatureControl::thermistor_read_tick );
    this->PIDdt= 1.0 / this->readings_per_second;

    // PID
    setPIDp( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, p_factor_checksum)->by_default(10 )->as_number() );
    setPIDi( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, i_factor_checksum)->by_default(0.3f)->as_number() );
    setPIDd( THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, d_factor_checksum)->by_default(200)->as_number() );
    // set to the same as max_pwm by default
    this->i_max = THEKERNEL->config->value(temperature_control_checksum, this->name_checksum, i_max_checksum   )->by_default(this->heater_pin.max_pwm())->as_number();
    this->iTerm = 0.0;
    this->lastInput= -1.0;
    this->last_reading = 0.0;
}

void TemperatureControl::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m) {
        // Get temperature
        if( gcode->m == this->get_m_code ){
            char buf[32]; // should be big enough for any status
            int n= snprintf(buf, sizeof(buf), "%s:%3.1f /%3.1f @%d ", this->designator.c_str(), this->get_temperature(), ((target_temperature == UNDEFINED)?0.0:target_temperature), this->o);
            gcode->txt_after_ok.append(buf, n);
            gcode->mark_as_taken();

        } else if (gcode->m == 301) {
            gcode->mark_as_taken();
            if (gcode->has_letter('S') && (gcode->get_value('S') == this->pool_index))
            {
                if (gcode->has_letter('P'))
                    setPIDp( gcode->get_value('P') );
                if (gcode->has_letter('I'))
                    setPIDi( gcode->get_value('I') );
                if (gcode->has_letter('D'))
                    setPIDd( gcode->get_value('D') );
                if (gcode->has_letter('X'))
                    this->i_max    = gcode->get_value('X');
            }
            //gcode->stream->printf("%s(S%d): Pf:%g If:%g Df:%g X(I_max):%g Pv:%g Iv:%g Dv:%g O:%d\n", this->designator.c_str(), this->pool_index, this->p_factor, this->i_factor/this->PIDdt, this->d_factor*this->PIDdt, this->i_max, this->p, this->i, this->d, o);
            gcode->stream->printf("%s(S%d): Pf:%g If:%g Df:%g X(I_max):%g O:%d\n", this->designator.c_str(), this->pool_index, this->p_factor, this->i_factor/this->PIDdt, this->d_factor*this->PIDdt, this->i_max, o);

        } else if (gcode->m == 303) {
            if (gcode->has_letter('E') && (gcode->get_value('E') == this->pool_index)) {
                gcode->mark_as_taken();
                float target = 150.0;
                if (gcode->has_letter('S')) {
                    target = gcode->get_value('S');
                    gcode->stream->printf("Target: %5.1f\n", target);
                }
                int ncycles= 8;
                if (gcode->has_letter('C')) {
                    ncycles= gcode->get_value('C');
                }
                gcode->stream->printf("Start PID tune, command is %s\n", gcode->command.c_str());
                this->pool->PIDtuner->begin(this, target, gcode->stream, ncycles);
            }

        } else if (gcode->m == 500 || gcode->m == 503){// M500 saves some volatile settings to config override file, M503 just prints the settings
            gcode->stream->printf(";PID settings:\nM301 S%d P%1.4f I%1.4f D%1.4f\n", this->pool_index, this->p_factor, this->i_factor/this->PIDdt, this->d_factor*this->PIDdt);
            gcode->mark_as_taken();

        } else if( ( gcode->m == this->set_m_code || gcode->m == this->set_and_wait_m_code ) && gcode->has_letter('S') ) {
            gcode->mark_as_taken();

            // Attach gcodes to the last block for on_gcode_execute
            if( THEKERNEL->conveyor->queue.size() == 0 ){
                THEKERNEL->call_event(ON_GCODE_EXECUTE, gcode );
            }else{
                Block* block = THEKERNEL->conveyor->queue.get_ref( THEKERNEL->conveyor->queue.size() - 1 );
                block->append_gcode(gcode);
            }
        }
    }
}

void TemperatureControl::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_m){
        if (((gcode->m == this->set_m_code) || (gcode->m == this->set_and_wait_m_code))
            && gcode->has_letter('S'))
        {
            float v = gcode->get_value('S');

            if (v == 0.0)
            {
                this->target_temperature = UNDEFINED;
                this->heater_pin.set(0);
            }
            else
            {
                this->set_desired_temperature(v);

                if( gcode->m == this->set_and_wait_m_code)
                {
                    THEKERNEL->pauser->take();
                    this->waiting = true;
                }
            }
        }
    }
}

void TemperatureControl::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(temperature_control_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return; // will be bed or hotend

    // ok this is targeted at us, so send back the requested data
    if(pdr->third_element_is(current_temperature_checksum)) {
        // this must be static as it will be accessed long after we have returned
        static struct pad_temperature temp_return;
        temp_return.current_temperature= this->get_temperature();
        temp_return.target_temperature= (target_temperature == UNDEFINED) ? 0 : this->target_temperature;
        temp_return.pwm= this->o;

        pdr->set_data_ptr(&temp_return);
        pdr->set_taken();
    }
}

void TemperatureControl::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(temperature_control_checksum)) return;

    if(!pdr->second_element_is(this->name_checksum)) return; // will be bed or hotend

    // ok this is targeted at us, so set the temp
    float t= *static_cast<float*>(pdr->get_data_ptr());
    this->set_desired_temperature(t);
    pdr->set_taken();
}

void TemperatureControl::set_desired_temperature(float desired_temperature)
{
    if (desired_temperature == 1.0)
        desired_temperature = preset1;
    else if (desired_temperature == 2.0)
        desired_temperature = preset2;

    target_temperature = desired_temperature;
    if (desired_temperature == 0.0)
        heater_pin.set((o = 0));
}

float TemperatureControl::get_temperature(){
    return last_reading;
}

float TemperatureControl::adc_value_to_temperature(int adc_value)
{
    if ((adc_value == 4095) || (adc_value == 0))
        return INFINITY;
    float r = r2 / ((4095.0 / adc_value) - 1.0);
    if (r1 > 0)
        r = (r1 * r) / (r1 - r);
    return (1.0 / (k + (j * log(r / r0)))) - 273.15;
}

uint32_t TemperatureControl::thermistor_read_tick(uint32_t dummy){
    int r = new_thermistor_reading();

    float temperature = adc_value_to_temperature(r);

    if (target_temperature > 0)
    {
        if ((r <= 1) || (r >= 4094))
        {
            this->min_temp_violated = true;
            target_temperature = UNDEFINED;
            heater_pin.set(0);
        }
        else
        {
            pid_process(temperature);
            if ((temperature > target_temperature) && waiting)
            {
                THEKERNEL->pauser->release();
                waiting = false;
            }
        }
    }
    else
    {
        heater_pin.set((o = 0));
    }
    last_reading = temperature;
    return 0;
}

/**
 * Based on https://github.com/br3ttb/Arduino-PID-Library
 */
void TemperatureControl::pid_process(float temperature)
{
    float error = target_temperature - temperature;

    this->iTerm += (error * this->i_factor);
    if (this->iTerm > this->i_max) this->iTerm = this->i_max;
    else if (this->iTerm < 0.0) this->iTerm = 0.0;

    if(this->lastInput < 0.0) this->lastInput= temperature; // set first time
    float d= (temperature - this->lastInput);

    // calculate the PID output
    // TODO does this need to be scaled by max_pwm/256? I think not as p_factor already does that
    this->o = (this->p_factor*error) + this->iTerm - (this->d_factor*d);

    if (this->o >= heater_pin.max_pwm())
        this->o = heater_pin.max_pwm();
    else if (this->o < 0)
        this->o = 0;

    this->heater_pin.pwm(this->o);
    this->lastInput= temperature;
}

int TemperatureControl::new_thermistor_reading()
{
    int last_raw = THEKERNEL->adc->read(&thermistor_pin);
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

void TemperatureControl::on_second_tick(void* argument)
{
    if (waiting)
        THEKERNEL->streams->printf("%s:%3.1f /%3.1f @%d\n", designator.c_str(), get_temperature(), ((target_temperature == UNDEFINED)?0.0:target_temperature), o);
}

void TemperatureControl::setPIDp(float p) {
    this->p_factor= p;
}

void TemperatureControl::setPIDi(float i) {
    this->i_factor= i*this->PIDdt;
}

void TemperatureControl::setPIDd(float d) {
    this->d_factor= d/this->PIDdt;
}
