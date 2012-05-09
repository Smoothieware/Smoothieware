/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

// TODO : THIS FILE IS LAME, MUST BE MADE MUCH BETTER

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
#include "TemperatureControl.h"
#include "libs/Pin.h"

TemperatureControl::TemperatureControl(){}

TemperatureControl::TemperatureControl(uint16_t name){
    this->name_checksum = name;
    this->error_count = 0; 
}

void TemperatureControl::on_module_loaded(){
    
    // We start not desiring any temp
    this->desired_adc_value = UNDEFINED;

    // Settings
    this->on_config_reload(this);

    this->acceleration_factor = 10;

    this->kernel->slow_ticker->attach( 20, this, &TemperatureControl::thermistor_read_tick );

    // Register for events
    this->register_for_event(ON_GCODE_EXECUTE); 
    this->register_for_event(ON_MAIN_LOOP); 

}

void TemperatureControl::on_main_loop(void* argument){ }

// Get configuration from the config file
void TemperatureControl::on_config_reload(void* argument){

    this->readings_per_second = this->kernel->config->value(temperature_control_checksum, this->name_checksum, readings_per_second_checksum)->by_default(5)->as_number();

    // Values are here : http://reprap.org/wiki/Thermistor
    this->r0   = 100000;
    this->t0   = 25;
    this->beta = 4066;
    this->vadc = 3.3;
    this->vcc  = 3.3;
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
    this->r0 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r0_checksum  )->by_default(100000)->as_number();               // Stated resistance eg. 100K
    this->t0 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, t0_checksum  )->by_default(25    )->as_number() + 273.15;      // Temperature at stated resistance, eg. 25C
    this->beta =                this->kernel->config->value(temperature_control_checksum, this->name_checksum, beta_checksum)->by_default(4066  )->as_number();               // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
    this->vadc =                this->kernel->config->value(temperature_control_checksum, this->name_checksum, vadc_checksum)->by_default(3.3   )->as_number();               // ADC Reference
    this->vcc  =                this->kernel->config->value(temperature_control_checksum, this->name_checksum, vcc_checksum )->by_default(3.3   )->as_number();               // Supply voltage to potential divider
    this->r1 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r1_checksum  )->by_default(0     )->as_number();
    this->r2 =                  this->kernel->config->value(temperature_control_checksum, this->name_checksum, r2_checksum  )->by_default(4700  )->as_number();
    
    // Thermistor math 
    this->k = this->r0 * exp( -this->beta / this->t0 );
    if( r1 > 0 ){ this->vs = r1 * this->vcc / ( r1 + r2 ); this->rs = r1 * r2 / ( r1 + r2 ); }else{ this->vs = this->vcc; this->rs = r2; } 

    // Thermistor pin for ADC readings
    this->thermistor_pin = this->kernel->config->value(temperature_control_checksum, this->name_checksum, thermistor_pin_checksum )->required()->as_pin();
    this->kernel->adc->enable_pin(this->thermistor_pin);

    // Heater pin
    this->heater_pin     =  this->kernel->config->value(temperature_control_checksum, this->name_checksum, heater_pin_checksum)->required()->as_pin()->as_output();
    this->heater_pin->set(0);

}

void TemperatureControl::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    // Set temperature without waiting
    if( gcode->has_letter('M') && gcode->get_value('M') == 104 && gcode->has_letter('S') ){
        this->set_desired_temperature(gcode->get_value('S')); 
    } 

    // Set temperature and wait
    if( gcode->has_letter('M') && gcode->get_value('M') == 109 && gcode->has_letter('S') ){
        this->set_desired_temperature(gcode->get_value('S'));
        
        // Pause 
        this->kernel->pauser->take(); 
        this->waiting = true; 
    
    } 

    // Get temperature
    if( gcode->has_letter('M') && gcode->get_value('M') == 105 ){
        gcode->stream->printf("get temperature: %f current:%f target:%f \r\n", this->get_temperature(), this->new_thermistor_reading(), this->desired_adc_value );
    } 
}

void TemperatureControl::set_desired_temperature(double desired_temperature){
    this->desired_adc_value = this->temperature_to_adc_value(desired_temperature);
}

double TemperatureControl::get_temperature(){
    double temp = this->new_thermistor_reading() ;
    return this->adc_value_to_temperature( this->new_thermistor_reading() );
}

double TemperatureControl::adc_value_to_temperature(double adc_value){
    double v = adc_value * this->vadc;            // Convert from 0-1 adc value to voltage
    double r = this->rs * v / ( this->vs - v );   // Resistance of thermistor
    return ( this->beta / log( r / this->k )) - 273.15;
}

double TemperatureControl::temperature_to_adc_value(double temperature){
    double r = this->r0 * exp( this->beta * ( 1 / (temperature + 273.15) -1 / this->t0 ) ); // Resistance of the thermistor 
    double v = this->vs * r / ( this->rs + r );                                             // Voltage at the potential divider
    return v / this->vadc * 1.00000;                                               // The ADC reading
}

uint32_t TemperatureControl::thermistor_read_tick(uint32_t dummy){
    if( this->desired_adc_value != UNDEFINED ){
        if( this->new_thermistor_reading() > this->desired_adc_value ){
            this->heater_pin->set(1); 
        }else{
            this->heater_pin->set(0); 
            if( this->waiting ){ 
                this->kernel->pauser->release();
                this->waiting = false; 
            }
        }
    }
}

double TemperatureControl::new_thermistor_reading(){
   
    double new_reading = double( double(this->kernel->adc->read(this->thermistor_pin) / double(1<<12) ) );

    if( this->queue.size() < 15 ){
        this->queue.push_back( new_reading );
        return new_reading;
    }else{
        double current_temp = this->average_adc_reading();
        double error = fabs(new_reading - current_temp); 
        if( error < 0.1 ){
            this->error_count = 0;
            double test;
            this->queue.pop_front(test); 
            this->queue.push_back( new_reading );
        }else{
            this->error_count++;
            if( this->error_count > 4 ){
                double test;
                this->queue.pop_front(test); 
            }
        } 
        return current_temp;
    }
}


double TemperatureControl::average_adc_reading(){
    double total;
    int j=0;
    int reading_index = this->queue.head;
    while( reading_index != this->queue.tail ){
        j++;
        total += this->queue.buffer[reading_index];
        reading_index = this->queue.next_block_index( reading_index );
    }
    return total / j;
}



