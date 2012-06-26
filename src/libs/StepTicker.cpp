/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/



using namespace std;
#include <vector>
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepTicker.h"
#include "system_LPC17xx.h" // mbed.h lib


StepTicker* global_step_ticker;

StepTicker::StepTicker(){
    global_step_ticker = this;
    LPC_TIM0->MR0 = 1000000;        // Initial dummy value for Match Register
    LPC_TIM0->MCR = 11;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 1;              // Enable interrupt
    this->debug = 0;
    NVIC_EnableIRQ(TIMER0_IRQn);    // Enable interrupt handler
    this->last_duration = 0;
}

void StepTicker::set_frequency( double frequency ){
    this->frequency = frequency;
    this->period = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }
}

void StepTicker::set_reset_delay( double seconds ){
    LPC_TIM0->MR1 = int(floor(double(SystemCoreClock/4)*( seconds )));  // SystemCoreClock/4 = Timer increments in a second
    this->delay = LPC_TIM0->MR1; 
    printf("setting to %f seconds, or mr1: %u \r\n", seconds, LPC_TIM0->MR1 );
}

// Add a stepper motor object to our list of steppers we must take care of
StepperMotor* StepTicker::add_stepper_motor(StepperMotor* stepper_motor){
    this->stepper_motors.push_back(stepper_motor);
    stepper_motor->step_ticker = this; 
    return stepper_motor;
}

inline void StepTicker::tick(){ 
    for(unsigned int i=0; i < this->stepper_motors.size(); i++){ 
        this->stepper_motors[i]->tick();
    }
}

inline void StepTicker::reset_tick(){
    for(unsigned int i=0; i < this->stepper_motors.size(); i++){
        this->stepper_motors[i]->step_pin->set(0);
    }
}

extern "C" void TIMER0_IRQHandler (void){
    uint32_t start_time = LPC_TIM0->TC;

    global_step_ticker->debug++;

    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
    uint32_t d = 0;
    uint32_t e = 0;
    uint32_t f = 0;


    // Do not get out of here before everything is nice and tidy ( ~0x00 is a big number )
    LPC_TIM0->MR0 = 1000000;
    LPC_TIM0->MR1 = 1000000;

    if((LPC_TIM0->IR >> 0) & 1){  // If interrupt register set for MR0
        LPC_TIM0->IR |= 1 << 0;   // Reset it 
        a = LPC_TIM0->TC;
        global_step_ticker->tick(); 
        b = LPC_TIM0->TC; } 


    // If interrupt register set for MR1, means TC>MR0
    if((LPC_TIM0->IR >> 1) & 1){          
        LPC_TIM0->IR |= 1 << 1;   // Reset it
        c = LPC_TIM0->TC;
        global_step_ticker->reset_tick(); 
        d = LPC_TIM0->TC;
    }

    // If we went over the duration an interrupt is supposed to last, we have a problem 
    // That can happen tipically when we change blocks, where more than usual computation is done
    // This can be OK, if we take notice of it, which we do now
    if( LPC_TIM0->TC > global_step_ticker->period && global_step_ticker->stepper_motors.size() >= 2 ){ // TODO : remove the size condition

        uint32_t start_tc = LPC_TIM0->TC;

        // How many ticks we want to skip ( this does not include the current tick, but we add the time we spent doing this computation last time )
        uint32_t ticks_to_skip = (  ( LPC_TIM0->TC + global_step_ticker->last_duration ) / global_step_ticker->period );

        // Next step is now to reduce this to how many steps we can *actually* skip
        uint32_t ticks_we_actually_can_skip = ticks_to_skip;
        for(unsigned int i=0; i < global_step_ticker->stepper_motors.size(); i++){
            StepperMotor* stepper = global_step_ticker->stepper_motors[i];
            if( stepper->moving ){ ticks_we_actually_can_skip = min( ticks_we_actually_can_skip, (uint32_t)((uint64_t)( (uint64_t)stepper->fx_ticks_per_step - (uint64_t)stepper->fx_counter ) >> 32) ); }
        }

        // If the number of ticks we can actually skip is smaller than the number we wanted to skip, there is something wrong in the settings
        if( ticks_we_actually_can_skip < ticks_to_skip ){
            printf("Error : the number of ticks we can actually skip is smaller than the number we wanted to skip, there is something wrong in the settings, you are trying to step too fast, or with a too high base frequency\r\n");
            // TODO : Handle this in another manner
        }

        // Adding to MR0 for this time is not enough, we must also increment the counters ourself artificially
        for(unsigned int i=0; i < global_step_ticker->stepper_motors.size(); i++){
            StepperMotor* stepper = global_step_ticker->stepper_motors[i];
            if( stepper->moving ){ stepper->fx_counter += (uint64_t)((uint64_t)(ticks_we_actually_can_skip)<<32); }
        }

        // When must we have our next MR0 ? ( +1 is here to account that we are actually a legit MR0 too, not only overtime )
        LPC_TIM0->MR0 = ( ticks_we_actually_can_skip + 1 ) * global_step_ticker->period;
        // TODO : What the fuck do we do with you ?
        LPC_TIM0->MR1 = global_step_ticker->delay;

        // This is so that we know how long this computation takes, and we can take it into account next time
        global_step_ticker->last_duration = LPC_TIM0->TC - start_tc;

        printf("tc:%5u ld:%5u prd:%5u tts:%5u twcas:%5u nmr0:%5u %5u|%5u %5u|%5u \r\n", start_tc, global_step_ticker->last_duration, global_step_ticker->period, ticks_to_skip, ticks_we_actually_can_skip, LPC_TIM0->MR0, a, b, c, d );


    }else{

        LPC_TIM0->MR0 = global_step_ticker->period;
        LPC_TIM0->MR1 = global_step_ticker->delay;


    }

    
    
    /*
    if( global_step_ticker->debug % 100000 == 0 && global_step_ticker->period < 1000 && global_step_ticker->debug > 20000 ){ 
        printf("rand  start:%4u mr:%5u/%5u tc:%4u [%4u.%4u|%4u.%4u] \r\n", start_time, LPC_TIM0->MR0, LPC_TIM0->MR1, LPC_TIM0->TC,a,b,c,d ); 
    } 
    if( LPC_TIM0->TC > global_step_ticker->period && global_step_ticker->period < 1000 && global_step_ticker->debug > 20000 ){ 
        printf("match start:%4u mr:%5u/%5u tc:%4u [%4u.%4u|%4u.%4u] stp:%4u/%4u \r\n", start_time, LPC_TIM0->MR0, LPC_TIM0->MR1, LPC_TIM0->TC,a,b,c,d, global_step_ticker->stepper_motors[0]->stepped, global_step_ticker->stepper_motors[0]->steps_to_move ); 
    } 
*/


    if( LPC_TIM0->TC > global_step_ticker->period ){ 
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }


}


