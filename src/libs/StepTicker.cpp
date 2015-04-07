/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "StepTicker.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
#include "gpio.h"
extern GPIO stepticker_debug_pin;
#endif


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
    LPC_TIM1->TCR = 0;              // Disable interrupt

    // Setup RIT timer
    LPC_SC->PCONP |= (1L<<16); // RIT Power
    LPC_SC->PCLKSEL1 &= ~(3L << 26); // Clear PCLK_RIT bits;
    LPC_SC->PCLKSEL1 |=  (1L << 26); // Set PCLK_RIT bits to 0x01;
    LPC_RIT->RICOMPVAL = (uint32_t)(((SystemCoreClock / 1000000L) * 1000)-1); // 1ms period
    LPC_RIT->RICOUNTER = 0;
    // Set counter clear/reset after interrupt
    LPC_RIT->RICTRL |= (2L); //RITENCLR
    LPC_RIT->RICTRL &= ~(8L); // disable
    //NVIC_SetVector(RIT_IRQn, (uint32_t)&_ritisr);

    // Default start values
    this->a_move_finished = false;
    this->do_move_finished = 0;
    this->unstep.reset();
    this->set_frequency(100000);
    this->set_reset_delay(100);
    this->set_acceleration_ticks_per_second(1000);
    this->num_motors= 0;
    this->active_motor.reset();
    this->tick_cnt= 0;
}

StepTicker::~StepTicker() {
}

//called when everythinf is setup and interrupts can start
void StepTicker::start() {
    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(RIT_IRQn);
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency ){
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock/4.0F)/frequency);  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }
}

// Set the reset delay
void StepTicker::set_reset_delay( float microseconds ){
    uint32_t delay = floorf((SystemCoreClock/4.0F)*(microseconds/1000000.0F));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = delay;
}

// this is the number of acceleration ticks per second
void StepTicker::set_acceleration_ticks_per_second(uint32_t acceleration_ticks_per_second) {
    uint32_t us= roundf(1000000.0F/acceleration_ticks_per_second); // period in microseconds
    LPC_RIT->RICOMPVAL = (uint32_t)(((SystemCoreClock / 1000000L) * us)-1); // us
    LPC_RIT->RICOUNTER = 0;
    LPC_RIT->RICTRL |= (8L); // Enable rit
}

// Synchronize the acceleration timer, and optionally schedule it to fire now
void StepTicker::synchronize_acceleration(bool fire_now) {
    LPC_RIT->RICOUNTER = 0;
    if(fire_now){
        NVIC_SetPendingIRQ(RIT_IRQn);
    }else{
        if(NVIC_GetPendingIRQ(RIT_IRQn)) {
            // clear pending interrupt so it does not interrupt immediately
            LPC_RIT->RICTRL |= 1L; // also clear the interrupt in case it fired
            NVIC_ClearPendingIRQ(RIT_IRQn);
        }
    }
}


// Call signal_move_finished() on each active motor that asked to be signaled. We do this instead of inside of tick() so that
// all tick()s are called before we do the move finishing
void StepTicker::signal_a_move_finished(){
     for (int motor = 0; motor < num_motors; motor++){
        if (this->active_motor[motor] && this->motor[motor]->is_move_finished){
            this->motor[motor]->signal_move_finished();
                // Theoretically this does nothing and the reason for it is currently unknown and/or forgotten
                // if(this->motor[motor]->moving == false){
                //     if (motor > 0){
                //         motor--;
                //         bitmask >>= 1;
                //     }
                // }
        }
    }
}

// Reset step pins on any motor that was stepped
inline void StepTicker::unstep_tick(){
    for (int i = 0; i < num_motors; i++) {
        if(this->unstep[i]){
            this->motor[i]->unstep();
        }
    }
    this->unstep.reset();
}

extern "C" void TIMER1_IRQHandler (void){
    LPC_TIM1->IR |= 1 << 0;
    StepTicker::global_step_ticker->unstep_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIMER0_IRQHandler (void){
    StepTicker::global_step_ticker->TIMER0_IRQHandler();
}

extern "C" void RIT_IRQHandler (void){
    LPC_RIT->RICTRL |= 1L;
    StepTicker::global_step_ticker->acceleration_tick();
}

extern "C" void PendSV_Handler(void) {
    StepTicker::global_step_ticker->PendSV_IRQHandler();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::PendSV_IRQHandler (void) {

    if(this->do_move_finished.load() > 0) {
        this->do_move_finished--;
        #ifdef STEPTICKER_DEBUG_PIN
        stepticker_debug_pin= 1;
        #endif

        this->signal_a_move_finished();

        #ifdef STEPTICKER_DEBUG_PIN
        stepticker_debug_pin= 0;
        #endif
    }
}

// run in RIT lower priority than PendSV
void  StepTicker::acceleration_tick() {
    // call registered acceleration handlers
    for (size_t i = 0; i < acceleration_tick_handlers.size(); ++i) {
        acceleration_tick_handlers[i]();
    }
}

void StepTicker::TIMER0_IRQHandler (void){
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;
    tick_cnt++; // count number of ticks

    // Step pins NOTE takes 1.2us when nothing to step, 1.8-2us for one motor stepped and 2.6us when two motors stepped, 3.167us when three motors stepped
    for (uint32_t motor = 0; motor < num_motors; motor++){
        // send tick to all active motors
        if(this->active_motor[motor] && this->motor[motor]->tick()){
            // we stepped so schedule an unstep
            this->unstep[motor]= 1;
        }
    }

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if( this->unstep.any()){
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }
    // just let it run it will fire every 143 seconds
    // else{
    //     LPC_TIM1->TCR = 0; // disable interrupt, no point in it running if nothing to do
    // }

    if(this->a_move_finished) {
        this->a_move_finished= false;
        this->do_move_finished++; // Note this is an atomic variable because it is updated in two interrupts of different priorities so can be pre-empted
    }

    // If a move finished in this tick, we have to tell the actuator to act accordingly
    if(this->do_move_finished.load() > 0){
        // we delegate the slow stuff to the pendsv handler which will run as soon as this interrupt exits
        //NVIC_SetPendingIRQ(PendSV_IRQn); this doesn't work
        SCB->ICSR = 0x10000000; // SCB_ICSR_PENDSVSET_Msk;
    }
}

// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* motor)
{
    this->motor.push_back(motor);
    this->num_motors= this->motor.size();
    return this->num_motors-1;
}

// activate the specified motor, must have been registered
void StepTicker::add_motor_to_active_list(StepperMotor* motor)
{
    bool enabled= active_motor.any(); // see if interrupt was previously enabled
    active_motor[motor->index]= 1;
    if(!enabled) {
        LPC_TIM0->TCR = 1;               // Enable interrupt
    }
}

// Remove a stepper from the list of active motors
void StepTicker::remove_motor_from_active_list(StepperMotor* motor)
{
    active_motor[motor->index]= 0;
    // If we have no motor to work on, disable the whole interrupt
    if(this->active_motor.none()){
        LPC_TIM0->TCR = 0;               // Disable interrupt
        tick_cnt= 0;
    }
}
