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

}

void TemperatureControl::on_main_loop(void* argument){
    if (this->min_temp_violated) {
        kernel->streams->printf("MINTEMP triggered on P%d.%d! check your thermistors!\n", this->thermistor_pin.port_number, this->thermistor_pin.pin);
        this->min_temp_violated = false;
    }
}

// Get configuration from the config file
void TemperatureControl::on_config_reload(void* argument){

    // General config
    this->set_m_code          = this->kernel->config->value(temperature_control_checksum, this->name_checksum, set_m_code_checksum)->by_default(104)->as_number();
    this->set_and_wait_m_code = this->kernel->config->value(temperature_control_checksum, this->name_checksum, set_and_wait_m_code_checksum)->by_default(109)->as_number();
    this->get_m_code          = this->kernel->config->value(temperature_control_checksum, this->name_checksum, get_m_code_checksum)->by_default(105)->as_number();
    this->readings_per_second = this->kernel->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(20)->as_number();

    this->designator          = this->kernel->config->value(temperature_control_checksum, this->name_checksum, designator_checksum)->by_default(string("T"))->as_string();

    // Values are here : http://reprap.org/wiki/Thermistor
    this->r0   = 100000;
    this->t0   = 25;
    this->beta = 4066;
    this->r1   = 0;
    this->r2   = 4700;

    // Preset values for various common types of thermistors
    ConfigValue* thermistor = this->kernel->config->value(temperature_control_checksum, this->name_checksum, thermistor_checksum);
    if(       thermistor->value.compare("EPCOS100K"    ) == 0 ){ // Default
    }else if( thermistor->value.compare("RRRF100K"     ) == 0 ){ this->beta = 3960;
    }else if( thermistor->value.compare("RRRF10K"      ) == 0 ){ this->beta = 3964; this->r0 = 10000; this->r1 = 680; this->r2 = 1600;
    }else if( thermistor->value.compare("Honeywell100K") == 0 ){ this->beta = 3974;
    }else if( thermistor->value.compare("Semitec"      ) == 0 ){ this->beta = 4267; }

    // Preset values are overriden by specified values
    this->r0 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r0_checksum  )->by_default(this->r0  )->as_number();               // Stated resistance eg. 100K
    this->t0 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, t0_checksum  )->by_default(this->t0  )->as_number();               // Temperature at stated resistance, eg. 25C
    this->beta =                this->kernel->config->value(temperature_control_checksum, this->name_checksum, beta_checksum)->by_default(this->beta)->as_number();               // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    this->r1 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r1_checksum  )->by_default(this->r1  )->as_number();
    this->r2 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r2_checksum  )->by_default(this->r2  )->as_number();

    this->preset1 =             this->kernel->config->value(temperature_control_checksum, this->name_checksum, preset1_checksum)->by_default(0)->as_number();
    this->preset2 =             this->kernel->config->value(temperature_control_checksum, this->name_checksum, preset2_checksum)->by_default(0)->as_number();


    // Thermistor math
    j = (1.0 / beta);
    k = (1.0 / (t0 + 273.15));

    // sigma-delta output modulation
    o = 0;

    // Thermistor pin for ADC readings
    this->thermistor_pin.from_string(this->kernel->config->value(temperature_control_checksum, this->name_checksum, thermistor_pin_checksum )->required()->as_string());
    this->kernel->adc->enable_pin(&thermistor_pin);

    // Heater pin
    this->heater_pin.from_string(    this->kernel->config->value(temperature_control_checksum, this->name_checksum, heater_pin_checksum)->required()->as_string())->as_output();
    this->heater_pin.max_pwm(        this->kernel->config->value(temperature_control_checksum, this->name_checksum, max_pwm_checksum)->by_default(255)->as_number() );
    this->heater_pin.set(0);

    set_low_on_debug(heater_pin.port_number, heater_pin.pin);

    // activate SD-DAC timer
    this->kernel->slow_ticker->attach(1000, &heater_pin, &Pwm::on_tick);

    // reading tick
    this->kernel->slow_ticker->attach( this->readings_per_second, this, &TemperatureControl::thermistor_read_tick );

    // PID
    this->p_factor = this->kernel->config->value(temperature_control_checksum, this->name_checksum, p_factor_checksum)->by_default(10 )->as_number();
    this->i_factor = this->kernel->config->value(temperature_control_checksum, this->name_checksum, i_factor_checksum)->by_default(0.3)->as_number();
    this->d_factor = this->kernel->config->value(temperature_control_checksum, this->name_checksum, d_factor_checksum)->by_default(200)->as_number();
    this->i_max    = this->kernel->config->value(temperature_control_checksum, this->name_checksum, i_max_checksum   )->by_default(255)->as_number();
    this->i = 0.0;
    this->last_reading = 0.0;
}

void TemperatureControl::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if (gcode->has_m)
    {
        // Get temperature
        if( gcode->m == this->get_m_code ){
            gcode->stream->printf("%s:%3.1f /%3.1f @%d ", this->designator.c_str(), this->get_temperature(), ((target_temperature == UNDEFINED)?0.0:target_temperature), this->o);
            gcode->add_nl = true;
        }
        if (gcode->m == 301)
        {
            if (gcode->has_letter('S') && (gcode->get_value('S') == this->pool_index))
            {
                if (gcode->has_letter('P'))
                    this->p_factor = gcode->get_value('P');
                if (gcode->has_letter('I'))
                    this->i_factor = gcode->get_value('I');
                if (gcode->has_letter('D'))
                    this->d_factor = gcode->get_value('D');
                if (gcode->has_letter('X'))
                    this->i_max    = gcode->get_value('X');
            }
            gcode->stream->printf("%s(S%d): Pf:%g If:%g Df:%g X(I_max):%g Pv:%g Iv:%g Dv:%g O:%d\n", this->designator.c_str(), this->pool_index, this->p_factor, this->i_factor, this->d_factor, this->i_max, this->p, this->i, this->d, o);
        }
        if (gcode->m == 303)
        {
            if (gcode->has_letter('S') && (gcode->get_value('S') == this->pool_index))
            {
                double target = 150.0;
                if (gcode->has_letter('P'))
                {
                    target = gcode->get_value('P');
                    gcode->stream->printf("Target: %5.1f\n", target);
                }
                gcode->stream->printf("Start PID tune, command is %s\n", gcode->command.c_str());
                this->pool->PIDtuner->begin(this, target, gcode->stream);
            }
        }

        // Attach gcodes to the last block for on_gcode_execute
        if( ( gcode->m == this->set_m_code || gcode->m == this->set_and_wait_m_code ) && gcode->has_letter('S') ){
            if( this->kernel->conveyor->queue.size() == 0 ){
                this->kernel->call_event(ON_GCODE_EXECUTE, gcode );
            }else{
                Block* block = this->kernel->conveyor->queue.get_ref( this->kernel->conveyor->queue.size() - 1 );
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
            double v = gcode->get_value('S');

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
                    this->kernel->pauser->take();
                    this->waiting = true;
                }
            }
        }
    }
}


void TemperatureControl::set_desired_temperature(double desired_temperature)
{
    if (desired_temperature == 1.0)
        desired_temperature = preset1;
    else if (desired_temperature == 2.0)
        desired_temperature = preset2;

    target_temperature = desired_temperature;
    if (desired_temperature == 0.0)
        heater_pin.set((o = 0));
}

double TemperatureControl::get_temperature(){
    return last_reading;
}

double TemperatureControl::adc_value_to_temperature(int adc_value)
{
    if ((adc_value == 4095) || (adc_value == 0))
        return INFINITY;
    double r = r2 / ((4095.0 / adc_value) - 1.0);
    if (r1 > 0)
        r = (r1 * r) / (r1 - r);
    return (1.0 / (k + (j * log(r / r0)))) - 273.15;
}

uint32_t TemperatureControl::thermistor_read_tick(uint32_t dummy){
    int r = new_thermistor_reading();

    double temperature = adc_value_to_temperature(r);

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
                kernel->pauser->release();
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

void TemperatureControl::pid_process(double temperature)
{
    double error = target_temperature - temperature;

    p  = error * p_factor;
    i += (error * this->i_factor);
    // d was imbued with oldest_raw earlier in new_thermistor_reading
    d = adc_value_to_temperature(d);
    d = (d - temperature) * this->d_factor;

    if (i > this->i_max)
        i = this->i_max;
    if (i < -this->i_max)
        i = -this->i_max;

    this->o = (p + i + d) * heater_pin.max_pwm() / 256;

    if (this->o >= heater_pin.max_pwm())
    {
        i = 0;
        this->o = heater_pin.max_pwm();
    }
    if (this->o < 0)
    {
        if (this->o < -(heater_pin.max_pwm()))
            i = 0;
        this->o = 0;
    }

    this->heater_pin.pwm(o);
}

int TemperatureControl::new_thermistor_reading()
{
    int last_raw = this->kernel->adc->read(&thermistor_pin);
    if (queue.size() >= queue.capacity())
    {
        uint16_t l;
        queue.pop_front(l);
        d = l;
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
        kernel->streams->printf("%s:%3.1f /%3.1f @%d\n", designator.c_str(), get_temperature(), ((target_temperature == UNDEFINED)?0.0:target_temperature), o);
}
