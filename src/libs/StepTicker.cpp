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
    LPC_TIM0->MR0 = 10000000;        // Initial dummy value for Match Register
    LPC_TIM0->MR1 = 10000000;
    LPC_TIM0->MCR = 11;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 1;              // Enable interrupt
    this->debug = 0;
    this->has_axes = 0;
    this->set_frequency(0.001);
    this->set_reset_delay(100);
    this->last_duration = 0;
    NVIC_EnableIRQ(TIMER0_IRQn);    // Enable interrupt handler
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
    this->delay = int(floor(double(SystemCoreClock/4)*( seconds )));  // SystemCoreClock/4 = Timer increments in a second
}

// Add a stepper motor object to our list of steppers we must take care of
StepperMotor* StepTicker::add_stepper_motor(StepperMotor* stepper_motor){
    this->stepper_motors.push_back(stepper_motor);
    stepper_motor->step_ticker = this; 
    this->has_axes = true;
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
    
    if( !global_step_ticker->has_axes ){ 
        if((LPC_TIM0->IR >> 0) & 1){ LPC_TIM0->IR |= 1 << 0; }
        if((LPC_TIM0->IR >> 1) & 1){ LPC_TIM0->IR |= 1 << 1; }
        return; 
    } 
    
    LPC_TIM0->MR1 = 2000000;

    if((LPC_TIM0->IR >> 0) & 1){  // If interrupt register set for MR0
        // Do not get out of here before everything is nice and tidy
   
        //if( global_step_ticker->debug % 1000 == 1 ){ printf("start: tc: %u mr0: %u mr1: %u d: %u\r\n", LPC_TIM0->TC, LPC_TIM0->MR0, LPC_TIM0->MR1, global_step_ticker->debug); }
        
        LPC_TIM0->MR0 = 2000000;
   
        LPC_TIM0->IR |= 1 << 0;   // Reset it 
        
        // Step pins 
        uint32_t aa = LPC_TIM0->TC; 
        global_step_ticker->tick(); 
        uint32_t bb = LPC_TIM0->TC; 
    
        // Maybe we have spent enough time in this interrupt so that we have to reset the pins ourself
        if( LPC_TIM0->TC > global_step_ticker->delay ){
            global_step_ticker->reset_tick(); 
        }else{
            // Else we have to trigger this a tad later, using MR1
            LPC_TIM0->MR1 = global_step_ticker->delay; 
        } 
        uint32_t cc = LPC_TIM0->TC; 

        // If we went over the duration an interrupt is supposed to last, we have a problem 
        // That can happen tipically when we change blocks, where more than usual computation is done
        // This can be OK, if we take notice of it, which we do now
        if( LPC_TIM0->TC > global_step_ticker->period ){ // TODO : remove the size condition

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
            if( ticks_we_actually_can_skip < ticks_to_skip ){ }

            // Adding to MR0 for this time is not enough, we must also increment the counters ourself artificially
            for(unsigned int i=0; i < global_step_ticker->stepper_motors.size(); i++){
                StepperMotor* stepper = global_step_ticker->stepper_motors[i];
                if( stepper->moving ){ stepper->fx_counter += (uint64_t)((uint64_t)(ticks_we_actually_can_skip)<<32); }
            }

            // When must we have our next MR0 ? ( +1 is here to account that we are actually a legit MR0 too, not only overtime )
            LPC_TIM0->MR0 = ( ticks_we_actually_can_skip + 1 ) * global_step_ticker->period;

            // This is so that we know how long this computation takes, and we can take it into account next time
            global_step_ticker->last_duration = LPC_TIM0->TC - start_tc;

            if( global_step_ticker->debug % 50 == 1 || 0 ){
                printf("a\r\n");              
                
                printf("tc:%5u ld:%5u prd:%5u tts:%5u twcas:%5u nmr0:%5u \r\n", start_tc, global_step_ticker->last_duration, global_step_ticker->period, ticks_to_skip, ticks_we_actually_can_skip,  LPC_TIM0->MR0 );
                for(unsigned int j=0; j < global_step_ticker->stepper_motors.size(); j++){
                    StepperMotor* stepper = global_step_ticker->stepper_motors[j];
                    if( stepper->moving ){ 
                        uint32_t dd = (uint32_t)((uint64_t)( (uint64_t)stepper->fx_ticks_per_step - (uint64_t)stepper->fx_counter ) >> 32) ;
                        printf("    %u: %u\r\n", j,  dd); 
                    }
                }
                

            }


        }else{
            LPC_TIM0->MR0 = global_step_ticker->period;
        }

    }else{
        // Else obviously it's MR1 
        LPC_TIM0->IR |= 1 << 1;   // Reset it
        // Reset pins 
        global_step_ticker->reset_tick(); 
    }

    //printf("end: tc: %u mr0: %u mr1: %u d: %u\r\n", LPC_TIM0->TC, LPC_TIM0->MR0, LPC_TIM0->MR1, global_step_ticker->debug);

    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }


}


