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

extern bool _isr_context;

// StepTicker handles the base frequency ticking for the Stepper Motors / Actuators
// It has a list of those, and calls their tick() functions at regular intervals
// They then do Bresenham stuff themselves

StepTicker* StepTicker::global_step_ticker;

StepTicker::StepTicker(int nmotors){
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
    this->a_move_finished = false;
    this->pending_sv = false;
    this->reset_step_pins = false;
    this->set_frequency(0.001);
    this->set_reset_delay(100);
    this->last_duration = 0;
    //this->overruns= 0;
    this->num_motors= nmotors;
    this->active_motors= new StepperMotor*[num_motors];
    for (int i = 0; i < num_motors; i++){
        this->active_motors[i] = nullptr;
    }
    this->active_motor_bm = 0;

    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
}

StepTicker::~StepTicker() {
    delete[] this->active_motors;
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
void StepTicker::set_reset_delay( float seconds ){
    this->delay = floorf((SystemCoreClock/4.0F)*seconds);  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = this->delay;
}

// Call tick() on each active motor
inline void StepTicker::tick(){
    _isr_context = true;
    int i;
    uint32_t bm = 1;
    // We iterate over each active motor
    for (i = 0; i < num_motors; i++, bm <<= 1){
        if (this->active_motor_bm & bm){
            this->active_motors[i]->tick();
        }
    }
    _isr_context = false;
}

// Call signal_move_finished() on each active motor that asked to be signaled. We do this instead of inside of tick() so that
// all tick()s are called before we do the move finishing
void StepTicker::signal_a_move_finished(){
    _isr_context = true;

    uint16_t bitmask = 1;
    for ( uint8_t motor = 0; motor < num_motors; motor++, bitmask <<= 1){
        if (this->active_motor_bm & bitmask){
            if(this->active_motors[motor]->is_move_finished){
                this->active_motors[motor]->signal_move_finished();
                // Theoretically this does nothing and the reason for it is currently unknown and/or forgotten
                // if(this->active_motors[motor]->moving == false){
                //     if (motor > 0){
                //         motor--;
                //         bitmask >>= 1;
                //     }
                // }
            }
        }
    }
    this->a_move_finished = false;

    _isr_context = false;
}

// Reset step pins on all active motors
inline void StepTicker::reset_tick(){
    _isr_context = true;

    int i;
    uint32_t bm;
    for (i = 0, bm = 1; i < num_motors; i++, bm <<= 1)
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

extern "C" void PendSV_Handler(void) {
    StepTicker::global_step_ticker->PendSV_IRQHandler();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::PendSV_IRQHandler (void){
    #ifdef STEPTICKER_DEBUG_PIN
    stepticker_debug_pin= 1;
    #endif

    this->signal_a_move_finished();

    this->pending_sv= false;

    #ifdef STEPTICKER_DEBUG_PIN
    stepticker_debug_pin= 0;
    #endif
}

void StepTicker::TIMER0_IRQHandler (void){
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;
    LPC_TIM0->MR0 = this->period;

    // Step pins
    for (uint32_t motor = 0, bm = 1; motor < num_motors; motor++, bm <<= 1){
        if (this->active_motor_bm & bm){
            this->active_motors[motor]->tick();
        }
    }

    // We may have set a pin on in this tick, now we start the timer to set it off
    if( this->reset_step_pins ){
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
        this->reset_step_pins = false;
    }else{
        return;
    }

    // If a move finished in this tick, we have to tell the actuator to act accordingly
    if(!this->pending_sv && this->a_move_finished ){
        this->pending_sv= true; // don't multiple trigger pendsv
        // we delegate the slow stuff to the pendsv handler which will run as soon as this interrupt exits
        //NVIC_SetPendingIRQ(PendSV_IRQn); this doesn't work
        SCB->ICSR = 0x10000000; // SCB_ICSR_PENDSVSET_Msk;
    }
}


// TODO optimize so we don't have to search, keep index in steppermotor
// We make a list of steppers that want to be called so that we don't call them for nothing
void StepTicker::add_motor_to_active_list(StepperMotor* motor)
{
    uint32_t bm;
    int i;
    for (i = 0, bm = 1; i < num_motors; i++, bm <<= 1)
    {
        if (this->active_motors[i] == motor)
        {
            this->active_motor_bm |= bm;
            if( this->active_motor_bm != 0 ){ // this is always true!
                LPC_TIM0->TCR = 1;               // Enable interrupt
            }
            return;
        }
        if (this->active_motors[i] == nullptr)
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
    for (i = 0, bm = 1; i < num_motors; i++, bm <<= 1)
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
