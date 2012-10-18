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

// StepTicker handles the base frequency ticking for the Stepper Motors / Actuators
// It has a list of those, and calls their tick() functions at regular intervals
// They then do Bresenham stuff themselves

StepTicker* global_step_ticker;

StepTicker::StepTicker(){
    global_step_ticker = this;
    LPC_TIM0->MR0 = 10000000;        // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 1;               // Enable interrupt

    LPC_SC->PCONP |= (1 << 2);     // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 1;
    LPC_TIM1->TCR = 1;               // Enable interrupt

    // Default start values 
    this->debug = 0;
    this->has_axes = 0;
    this->set_frequency(0.001);
    this->set_reset_delay(100);
    this->last_duration = 0;
    this->active_motors[0] = NULL;   

    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
}

// Set the base stepping frequency
void StepTicker::set_frequency( double frequency ){
    this->frequency = frequency;
    this->period = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }
}

// Set the reset delay
void StepTicker::set_reset_delay( double seconds ){
    this->delay = int(floor(double(SystemCoreClock/4)*( seconds )));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = this->delay;
}

// Add a stepper motor object to our list of steppers we must take care of
StepperMotor* StepTicker::add_stepper_motor(StepperMotor* stepper_motor){
    this->stepper_motors.push_back(stepper_motor);
    stepper_motor->step_ticker = this; 
    this->has_axes = true;
    return stepper_motor;
}

inline void StepTicker::tick(){ 
    uint8_t current_id = 0; 
    StepperMotor* current = this->active_motors[0];
    while(current != NULL ){
        current->tick(); 
        current_id++;
        current = this->active_motors[current_id];
    }
}

inline void StepTicker::reset_tick(){
    
    uint8_t current_id = 0; 
    StepperMotor* current = this->active_motors[0];
    while(current != NULL ){
        current->step_pin->set(0);
        if( current->exit_tick ){
            if( current->dont_remove_from_active_list_yet ){
                current->dont_remove_from_active_list_yet = false;
            }else{
                this->remove_motor_from_active_list(current); 
                current_id--;
            }
        }
        current_id++;
        current = this->active_motors[current_id];
    }
}

extern "C" void TIMER1_IRQHandler (void){
    LPC_TIM1->IR |= 1 << 0; 
    global_step_ticker->reset_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIMER0_IRQHandler (void){
    
    uint32_t start_time = LPC_TIM0->TC;
   
    LPC_TIM0->IR |= 1 << 0;
   
    global_step_ticker->debug++;

    // If no axes enabled, just ignore for now 
    if( !global_step_ticker->has_axes ){ 
        return; 
    } 

    // Do not get out of here before everything is nice and tidy
    LPC_TIM0->MR0 = 2000000;

    // Step pins 
    global_step_ticker->tick(); 

    LPC_TIM1->TCR = 3;
    LPC_TIM1->TCR = 1;

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
        //if( ticks_we_actually_can_skip < ticks_to_skip ){ }

        // Adding to MR0 for this time is not enough, we must also increment the counters ourself artificially
        for(unsigned int i=0; i < global_step_ticker->stepper_motors.size(); i++){
            StepperMotor* stepper = global_step_ticker->stepper_motors[i];
            if( stepper->moving ){ stepper->fx_counter += (uint64_t)((uint64_t)(ticks_we_actually_can_skip)<<32); }
        }

        // When must we have our next MR0 ? ( +1 is here to account that we are actually a legit MR0 too, not only overtime )
        LPC_TIM0->MR0 = ( ticks_we_actually_can_skip + 1 ) * global_step_ticker->period;
        //LPC_TIM0->MR0 = ( ticks_to_skip + 1 ) * global_step_ticker->period;

        // This is so that we know how long this computation takes, and we can take it into account next time
        global_step_ticker->last_duration = LPC_TIM0->TC - start_tc;


    }else{
        LPC_TIM0->MR0 = global_step_ticker->period;
    }



    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
    //    LPC_TIM0->TCR = 3;  // Reset
    //    LPC_TIM0->TCR = 1;  // Reset
        LPC_TIM0->TC = LPC_TIM0->MR0 - 2;
    }



}


// We make a list of steppers that want to be called so that we don't call them for nothing
void StepTicker::add_motor_to_active_list(StepperMotor* motor){
    //printf("adding %p with exit: %u \r\n", motor, motor->exit_tick );
    uint8_t current_id = 0; 
    StepperMotor* current = this->active_motors[0];
    while(current != NULL ){
        if( current == motor ){ 
            return;  // Motor already in list
        }
        current_id++;
        current = this->active_motors[current_id];
    }
    this->active_motors[current_id  ] = motor;
    this->active_motors[current_id+1] = NULL;

}

void StepTicker::remove_motor_from_active_list(StepperMotor* motor){
    //printf("removing %p with exit: %u \r\n", motor, motor->exit_tick );
    uint8_t current_id = 0;
    uint8_t offset = 0; 
    StepperMotor* current = this->active_motors[0];
    while( current != NULL ){
        if( current == motor ){ offset++; }
        this->active_motors[current_id] = this->active_motors[current_id+offset];
        current_id++;
        current = this->active_motors[current_id+offset];
    }
    this->active_motors[current_id] = NULL;


}


