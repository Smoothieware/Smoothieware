/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "StepTicker.h"

using namespace std;
#include <vector>

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

extern bool _isr_context;

// StepTicker handles the base frequency ticking for the Stepper Motors / Actuators
// It has a list of those, and calls their tick() functions at regular intervals
// They then do Bresenham stuff themselves

StepTicker* StepTicker::global_step_ticker;

StepTicker::StepTicker(){
    StepTicker::global_step_ticker = this;

    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 0;              // Disable interrupt

    LPC_SC->PCONP |= (1 << 2);      // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 1;
    LPC_TIM1->TCR = 1;              // Enable interrupt

    // Default start values
    this->moves_finished = false;
    this->reset_step_pins = false;
    this->debug = 0;
    this->has_axes = 0;
    this->set_frequency(0.001);
    this->set_reset_delay(100);
    this->last_duration = 0;
    for (int i = 0; i < 12; i++){
        this->active_motors[i] = NULL;
    }
    this->active_motor_bm = 0;

    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency ){
    this->frequency = frequency;
    this->period = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }
}

// Set the reset delay
void StepTicker::set_reset_delay( float seconds ){
    this->delay = int(floor(float(SystemCoreClock/4)*( seconds )));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = this->delay;
}

// Add a stepper motor object to our list of steppers we must take care of
StepperMotor* StepTicker::add_stepper_motor(StepperMotor* stepper_motor){
    this->stepper_motors.push_back(stepper_motor);
    stepper_motor->step_ticker = this;
    this->has_axes = true;
    return stepper_motor;
}

// Call tick() on each active motor
inline void StepTicker::tick(){
    _isr_context = true;
    int i;
    uint32_t bm = 1;
    // We iterate over each active motor
    for (i = 0; i < 12; i++, bm <<= 1){
        if (this->active_motor_bm & bm){
            this->active_motors[i]->tick();
        }
    }
    _isr_context = false;
}

// Call signal_mode_finished() on each active motor that asked to be signaled. We do this instead of inside of tick() so that
// all tick()s are called before we do the move finishing
void StepTicker::signal_moves_finished(){
    _isr_context = true;

    uint16_t bitmask = 1;
    for ( uint8_t motor = 0; motor < 12; motor++, bitmask <<= 1){
        if (this->active_motor_bm & bitmask){
            if(this->active_motors[motor]->is_move_finished){
                this->active_motors[motor]->signal_move_finished();
                if(this->active_motors[motor]->moving == false){
                    if (motor > 0){
                        motor--;
                        bitmask >>= 1;
                    }
                }
            }
        }
    }
    this->moves_finished = false;

    _isr_context = false;
}

// Reset step pins on all active motors
inline void StepTicker::reset_tick(){
    _isr_context = true;

    int i;
    uint32_t bm;
    for (i = 0, bm = 1; i < 12; i++, bm <<= 1)
    {
        if (this->active_motor_bm & bm)
            this->active_motors[i]->unstep();
    }

    _isr_context = false;
}

extern "C" void TIMER1_IRQHandler (void){
    LPC_TIM1->IR |= 1 << 0;
    StepTicker::global_step_ticker->reset_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIMER0_IRQHandler (void){
    StepTicker::global_step_ticker->TIMER0_IRQHandler();
}

void StepTicker::TIMER0_IRQHandler (void){
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;

    // Step pins
    uint16_t bitmask = 1;
    for (uint8_t motor = 0; motor < 12; motor++, bitmask <<= 1){
        if (this->active_motor_bm & bitmask){
            this->active_motors[motor]->tick();
        }
    }

    // We may have set a pin on in this tick, now we start the timer to set it off
    if( this->reset_step_pins ){
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
        this->reset_step_pins = false;
    }else{
        // Nothing happened, nothing after this really matters
        // TODO : This could be a problem when we use Actuators instead of StepperMotors, because this flag is specific to step generation
        LPC_TIM0->MR0 = this->period;
        return;
    }

    // If a move finished in this tick, we have to tell the actuator to act accordingly
    if( this->moves_finished ){

        // Do not get out of here before everything is nice and tidy
        LPC_TIM0->MR0 = 20000000;

        this->signal_moves_finished();

        // If we went over the duration an interrupt is supposed to last, we have a problem
        // That can happen tipically when we change blocks, where more than usual computation is done
        // This can be OK, if we take notice of it, which we do now
        if( LPC_TIM0->TC > this->period ){ // TODO: remove the size condition

            uint32_t start_tc = LPC_TIM0->TC;

            // How many ticks we want to skip ( this does not include the current tick, but we add the time we spent doing this computation last time )
            uint32_t ticks_to_skip = (  ( LPC_TIM0->TC + this->last_duration ) / this->period );

            // Next step is now to reduce this to how many steps we can *actually* skip
            uint32_t ticks_we_actually_can_skip = ticks_to_skip;

            int i;
            uint32_t bm;
            for (i = 0, bm = 1; i < 12; i++, bm <<= 1)
            {
                if (this->active_motor_bm & bm)
                    ticks_we_actually_can_skip =
                        min(ticks_we_actually_can_skip,
                            (uint32_t)((uint64_t)( (uint64_t)this->active_motors[i]->fx_ticks_per_step - (uint64_t)this->active_motors[i]->fx_counter ) >> 32)
                            );
            }

            // Adding to MR0 for this time is not enough, we must also increment the counters ourself artificially
            for (i = 0, bm = 1; i < 12; i++, bm <<= 1)
            {
                if (this->active_motor_bm & bm)
                    this->active_motors[i]->fx_counter += (uint64_t)((uint64_t)(ticks_we_actually_can_skip)<<32);
            }

            // When must we have our next MR0 ? ( +1 is here to account that we are actually doing a legit MR0 match here too, not only overtime )
            LPC_TIM0->MR0 = ( ticks_to_skip + 1 ) * this->period;

            // This is so that we know how long this computation takes, and we can take it into account next time
            int difference = (int)(LPC_TIM0->TC) - (int)(start_tc);
            if( difference > 0 ){ this->last_duration = (uint32_t)difference; }

        }else{
            LPC_TIM0->MR0 = this->period;
        }

        while( LPC_TIM0->TC > LPC_TIM0->MR0 ){
            LPC_TIM0->MR0 += this->period;
        }

    }

}


// We make a list of steppers that want to be called so that we don't call them for nothing
void StepTicker::add_motor_to_active_list(StepperMotor* motor)
{
    uint32_t bm;
    int i;
    for (i = 0, bm = 1; i < 12; i++, bm <<= 1)
    {
        if (this->active_motors[i] == motor)
        {
            this->active_motor_bm |= bm;
            if( this->active_motor_bm != 0 ){
                LPC_TIM0->TCR = 1;               // Enable interrupt
            }
            return;
        }
        if (this->active_motors[i] == NULL)
        {
            this->active_motors[i] = motor;
            this->active_motor_bm |= bm;
            if( this->active_motor_bm != 0 ){
                LPC_TIM0->TCR = 1;               // Enable interrupt
            }
            return;
        }
    }
    return;
}

// Remove a stepper from the list of active motors
void StepTicker::remove_motor_from_active_list(StepperMotor* motor)
{
    uint32_t bm; int i;
    for (i = 0, bm = 1; i < 12; i++, bm <<= 1)
    {
        if (this->active_motors[i] == motor)
        {
            this->active_motor_bm &= ~bm;
            // If we have no motor to work on, disable the whole interrupt
            if( this->active_motor_bm == 0 ){
                LPC_TIM0->TCR = 0;               // Disable interrupt
            }
            return;
        }
    }
}
