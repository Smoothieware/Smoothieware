/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "mbed.h"
#include "libs/Kernel.h"
#include "modules/tools/laser/Laser.h"
#include "modules/tools/extruder/Extruder.h"
#include "modules/robot/Player.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/pauser/Pauser.h"
#include "libs/SDFileSystem.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

SDFileSystem sd(p5, p6, p7, p8, "sd");
//LocalFileSystem local("local");

#include <math.h>
#define UNDEFINED -1
class TemperatureControl : public Module {
    public:
        TemperatureControl(){
            this->error_count = 0; 
        }

        void on_module_loaded(){
            
            // We start now desiring any temp
            this->desired_adc_value = UNDEFINED;

            // Settings
            this->readings_per_second = 50;

            this->r0 = 100000;               // Stated resistance eg. 100K
            this->t0 = 25 + 273.15;          // Temperature at stated resistance, eg. 25C
            this->beta = 4066;               // Thermistor beta rating. See http://reprap.org/bin/view/Main/MeasuringThermistorBeta
            this->vadc = 3.3;                // ADC Reference
            this->vcc  = 3.3;                // Supply voltage to potential divider
            this->k = this->r0 * exp( -this->beta / this->t0 );
            double r1 = 0;
            double r2 = 4700;

            if( r1 > 0 ){
                this->vs = r1 * this->vcc / ( r1 + r2 );
                this->rs = r1 * r2 / ( r1 + r2 );
            }else{
                this->vs = this->vcc;
                this->rs = r2;
            } 

            this->acceleration_factor = 10;

            // Setup pins and timer 
            this->thermistor_pin = new AnalogIn(p20); 
            this->thermistor_read_ticker = new Ticker();
            this->thermistor_read_ticker->attach(this, &TemperatureControl::thermistor_read_tick, 1/this->readings_per_second);
            this->heater_pwm = new PwmOut(p22);
            this->heater_pwm->write(0);
            this->pwm_value = 0;

            // Register for events
            this->register_for_event(ON_GCODE_EXECUTE); 

        }

        void on_gcode_execute(void* argument){
            Gcode* gcode = static_cast<Gcode*>(argument);
            
            // Set temperature
            if( gcode->has_letter('M') && gcode->get_value('M') == 104 && gcode->has_letter('S') ){
                this->set_desired_temperature(gcode->get_value('S')); 
            } 
       
            // Get temperature
            if( gcode->has_letter('M') && gcode->get_value('M') == 105 ){
                this->kernel->serial->printf("get temperature: %f \r\n", this->get_temperature() );
            } 
        }

        void set_desired_temperature(double desired_temperature){
            this->desired_adc_value = this->temperature_to_adc_value(desired_temperature);
            this->tail_adc_value =  this->temperature_to_adc_value(desired_temperature-20);
            this->head_adc_value =  this->temperature_to_adc_value(desired_temperature+5);
            //this->kernel->serial->printf("requested temperature: %f, adc value: %f, k: %f \r\n", desired_temperature, this->desired_adc_value, this->k );
        }

        double get_temperature(){
            double temp = this->new_thermistor_reading() ;
            //this->kernel->serial->printf("adc reading: %f \r\n", temp); 
            return this->adc_value_to_temperature( this->new_thermistor_reading() );
        }

        double adc_value_to_temperature(double adc_value){
            double v = adc_value * this->vadc;            // Convert from 0-1 adc value to voltage
            double r = this->rs * v / ( this->vs - v );   // Resistance of thermistor
            return ( this->beta / log( r / this->k )) - 273.15;
        }

        double temperature_to_adc_value(double temperature){
            double r = this->r0 * exp( this->beta * ( 1 / (temperature + 273.15) -1 / this->t0 ) ); // Resistance of the thermistor 
            double v = this->vs * r / ( this->rs + r );                                             // Voltage at the potential divider
            return v / this->vadc * 1.00000;                                               // The ADC reading
        }

        void thermistor_read_tick(){
            double reading = this->new_thermistor_reading();
            if( this->desired_adc_value != UNDEFINED ){
                double difference = fabs( reading - this->desired_adc_value ); 
                double adjustment = difference / acceleration_factor / this->readings_per_second;
                if( reading > this->tail_adc_value ){
                    this->heater_pwm->write( 1 );
                    //this->kernel->serial->printf("under: %f \r\n", this->adc_value_to_temperature(reading) ); 
                }else if( reading < this->head_adc_value ){
                    this->pwm_value -= adjustment;
                    this->heater_pwm->write( 0 );
                    //this->kernel->serial->printf("over: %f \r\n", this->adc_value_to_temperature(reading) ); 
                }else{
                   if( reading > this->desired_adc_value ){
                        this->pwm_value += adjustment;  // Heat up
                    }else{
                        this->pwm_value -= adjustment;  // Heat down
                    }
                    this->pwm_value = max( double(0), min( double(1), pwm_value ) );
                    this->heater_pwm->write( pwm_value ); 
                    //this->kernel->serial->printf("temp: %f \r\n", this->adc_value_to_temperature(reading) ); 
                    //this->kernel->serial->printf("read:%.5f, des_adc_val:%.5ff, diff: %.5f, ad: %f, pwm:%f \r\n", reading, this->desired_adc_value, difference, adjustment, pwm_value );
                }
            }
        }

        double new_thermistor_reading(){
            double new_reading = this->thermistor_pin->read();

            if( this->queue.size() < 15 ){
                this->queue.push_back( new_reading );
                //this->kernel->serial->printf("first\r\n");
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
                //this->kernel->serial->printf("read: %f temp: %f average: %f error: %f queue: %d \r\n",this->adc_value_to_temperature( new_reading ),  this->adc_value_to_temperature(current_temp), current_temp, error, this->queue.size() );
            }
        
        
        
        }

        double average_adc_reading(){
            double total;
            int j=0;
            for( int i = 0; i <= this->queue.size()-1; i++ ){
                j++; 
                //this->kernel->serial->printf("value:%f\r\n", *(this->queue.get_ref(i)) ); 
                total += *(this->queue.get_ref(i));
            }
            //this->kernel->serial->printf("total:%f, size:%d \r\n", total, this->queue.size() ); 
            return total / j;
        }

        AnalogIn* thermistor_pin;
        Ticker*   thermistor_read_ticker;
        Ticker*   debug_ticker;
        PwmOut*   heater_pwm;
        double    pwm_value; 
        double    desired_adc_value;
        double    tail_adc_value;
        double    head_adc_value;

        // Thermistor computation settings
        double r0;
        double t0;
        double beta;
        double vadc;
        double vcc;
        double k;
        double vs;
        double rs;
        
        double acceleration_factor;
        double readings_per_second;

        RingBuffer<double,16> queue;  // Queue of Blocks
        int error_count;
};


int main() {

    Kernel* kernel = new Kernel();

    kernel->serial->printf("Smoothie ( grbl port ) version 0.4 \r\nstart\r\n");

    kernel->add_module( new Laser(p21) );
    kernel->add_module( new Extruder(p26,p27) );
    kernel->add_module( new SimpleShell() );
    //kernel->add_module( new Pauser(p29,p30) );
    kernel->add_module( new TemperatureControl() );

    while(1){
        kernel->call_event(ON_MAIN_LOOP);

    }
}

