/*
 * TemperatureMonitor.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: mmoore
 */

#include "TemperatureMonitor.h"
#include "Kernel.h"
#include "StreamOutputPool.h"

#include <math.h>


#define TEMPERATURE_INTEGER_MASK   0x3F
#define TEMPERATURE_TO_INTEGER(t)  ((uint8_t)(((uint32_t)t) & TEMPERATURE_INTEGER_MASK))


TemperatureMonitor::TemperatureMonitor(uint8_t range, uint16_t timeout)
{
    this->state = NOT_HEATING;
    this->heating_timer = 0;
    this->heating_timeout = timeout;
    this->range = range;
    this->last_temperature = 0;
}

TemperatureMonitor::~TemperatureMonitor()
{
    // TODO Auto-generated destructor stub
}

void TemperatureMonitor::set_target_temperature(float target, float last_target, float current_temp)
{
    // set the runaway state.
    this->last_temperature = TEMPERATURE_TO_INTEGER(current_temp);
    this->heating_timer = 0;

    if ( target <= 0.0F ) {
        this->state = NOT_HEATING;
    } else if ( target < last_target ) {
        this->state = COOLING_DOWN;
    } else if ( target > last_target ) {
        this->state = HEATING_UP;
    }
}

void TemperatureMonitor::on_second_tick(float target_temp, float current_temp, string control_designator)
{
    if( target_temp <= 0 ){ // If we are not trying to heat, state is NOT_HEATING
        this->state = NOT_HEATING;
    }else{
        uint8_t current_temp_as_integer = TEMPERATURE_TO_INTEGER(current_temp);
        switch( this->state ){
            case NOT_HEATING: // handle the case where the temperature control is idle.
                if ( target_temp > 0 ) {
                    // If we were previously not trying to heat, but we are now, change to state COOLING_DOWN or HEATING_UP
                    this->state = (target_temp < current_temp) ? COOLING_DOWN : HEATING_UP;
                    this->heating_timer = 0;
                    this->last_temperature = current_temp_as_integer;
                }
                break;

            case HEATING_UP:     // handle the case where temperature has to be increased to reach the target.
            case COOLING_DOWN: { // handle the case where temperature has to be decreased to reach the target.
                bool target_reached = false;
                bool trending_correctly = true;
                // check whether the temperature is trending in the right direction.
                if ( this->state == HEATING_UP ) {
                    target_reached = (current_temp >= target_temp);
                    trending_correctly = (current_temp_as_integer > this->last_temperature);
                }
                else if ( this->state == COOLING_DOWN ) {
                    target_reached = (current_temp <= target_temp);
                    trending_correctly = (current_temp_as_integer < this->last_temperature);
                }

                if ( target_reached ) {
                    // the temperature has been reached, change to state MAINTAINING_TEMPERATURE
                    this->heating_timer = 0;
                    this->state = MAINTAINING_TEMPERATURE;
                }

                // as long as the temperature is trending correctly, all is well.
                // if it stalls, tick the heating timer.
                if ( trending_correctly ) {
                    // temperature is trending, reset the timer.
                    this->heating_timer = 0;
                } else {
                    // temperature is stalled, tick the timer.
                    this->heating_timer++;
                }
                this->last_temperature = current_temp_as_integer;

                // check whether the temperature has stalled longer than the timeout period.
                if(this->heating_timer > this->heating_timeout && this->heating_timeout != 0) {
                    this->heating_timer = 0;
                    THEKERNEL->streams->printf("ERROR: Temperature took too long to be reached on %s, HALT asserted, TURN POWER OFF IMMEDIATELY - reset or M999 required\n", control_designator.c_str());
                    THEKERNEL->call_event(ON_HALT, nullptr);
                }
                break;
            }
            case MAINTAINING_TEMPERATURE: { // handle the case where the target has been reached and temperature is being maintained.
                // check for thermal runaway
                float delta= current_temp - target_temp;

                // If the temperature is outside the acceptable range
                if(this->range != 0 && fabsf(delta) > this->range){
                    THEKERNEL->streams->printf("ERROR: Temperature runaway on %s (delta temp %f), HALT asserted, TURN POWER OFF IMMEDIATELY - reset or M999 required\n", control_designator.c_str(), delta);
                    THEKERNEL->call_event(ON_HALT, nullptr);
                }
            }
                break;
        }
    }
}
