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
#include "Block.h"
#include "Conveyor.h"

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
// debug pins, only used if defined in src/makefile
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

StepTicker *StepTicker::instance;

StepTicker::StepTicker()
{
    instance = this; // setup the Singleton instance of the stepticker

    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM0->TCR = 0;              // Disable interrupt

    LPC_SC->PCONP |= (1 << 2);      // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 5;              // match on Mr0, stop on match
    LPC_TIM1->TCR = 0;              // Disable interrupt

    // Default start values
    this->set_frequency(100000);
    this->set_unstep_time(100);

    this->unstep.reset();
    this->num_motors = 0;

    this->running = false;
    this->current_block = nullptr;

    #ifdef STEPTICKER_DEBUG_PIN
    // setup debug pin if defined
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

StepTicker::~StepTicker()
{
}

//called when everything is setup and interrupts can start
void StepTicker::start()
{
    NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
    NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock / 4.0F) / frequency); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM0->MR0 = this->period;
    LPC_TIM0->TCR = 3;  // Reset
    LPC_TIM0->TCR = 1;  // start
}

// Set the reset delay, must be called after set_frequency
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf((SystemCoreClock / 4.0F) * (microseconds / 1000000.0F)); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = delay;

    // TODO check that the unstep time is less than the step period, if not slow down step ticker
}

// Reset step pins on any motor that was stepped
void StepTicker::unstep_tick()
{
    for (int i = 0; i < num_motors; i++) {
        if(this->unstep[i]) {
            this->motor[i]->unstep();
        }
    }
    this->unstep.reset();
}

extern "C" void TIMER1_IRQHandler (void)
{
    LPC_TIM1->IR |= 1 << 0;
    StepTicker::getInstance()->unstep_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIMER0_IRQHandler (void)
{
    // Reset interrupt register
    LPC_TIM0->IR |= 1 << 0;
    StepTicker::getInstance()->step_tick();
}

extern "C" void PendSV_Handler(void)
{
    StepTicker::getInstance()->handle_finish();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::handle_finish (void)
{
    // all moves finished signal block is finished
    if(finished_fnc) finished_fnc();
}

// step clock
void StepTicker::step_tick (void)
{
    static uint32_t current_tick = 0;

    //SET_STEPTICKER_DEBUG_PIN(running ? 1 : 0);

    // if nothing has been setup we ignore the ticks
    if(!running){
        // check if anything new available
        if(THECONVEYOR->get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue
            if(!running) return;
        }else{
            return;
        }
    }

    if(THEKERNEL->is_halted()) {
        running= false;
        return;
    }

    bool still_moving= false;
    // foreach motor, if it is active see if time to issue a step to that motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue; // not active

        current_block->tick_info[m].steps_per_tick += current_block->tick_info[m].acceleration_change;

        if(current_tick == current_block->tick_info[m].next_accel_event) {
            if(current_tick == current_block->accelerate_until) { // We are done accelerating, deceleration becomes 0 : plateau
                current_block->tick_info[m].acceleration_change = 0;
                if(current_block->decelerate_after < current_block->total_move_ticks) {
                    current_block->tick_info[m].next_accel_event = current_block->decelerate_after;
                    if(current_tick != current_block->decelerate_after) { // We are plateauing
                        // steps/sec / tick frequency to get steps per tick
                        current_block->tick_info[m].steps_per_tick = current_block->tick_info[m].plateau_rate;
                    }
                }
            }

            if(current_tick == current_block->decelerate_after) { // We start decelerating
                current_block->tick_info[m].acceleration_change = current_block->tick_info[m].deceleration_change;
            }
        }

        // protect against rounding errors and such
        if(current_block->tick_info[m].steps_per_tick <= 0) {
            current_block->tick_info[m].counter = STEPTICKER_FPSCALE; // we force completion this step by setting to 1.0
            current_block->tick_info[m].steps_per_tick = 0;
        }

        current_block->tick_info[m].counter += current_block->tick_info[m].steps_per_tick;

        if(current_block->tick_info[m].counter >= STEPTICKER_FPSCALE) { // >= 1.0 step time
            current_block->tick_info[m].counter -= STEPTICKER_FPSCALE; // -= 1.0F;
            ++current_block->tick_info[m].step_count;

            // step the motor
            bool ismoving= motor[m]->step(); // returns false if the moving flag was set to false externally (probes, endstops etc)
            // we stepped so schedule an unstep
            unstep.set(m);

            if(!ismoving || current_block->tick_info[m].step_count == current_block->tick_info[m].steps_to_move) {
                // done
                current_block->tick_info[m].steps_to_move = 0;
                motor[m]->stop_moving(); // let motor know it is no longer moving
            }
        }

        // see if any motors are still moving after this tick
        if(motor[m]->is_moving()) still_moving= true;
    }

    // do this after so we start at tick 0
    current_tick++; // count number of ticks

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if( unstep.any()) {
        LPC_TIM1->TCR = 3;
        LPC_TIM1->TCR = 1;
    }


    // see if any motors are still moving
    if(!still_moving) {
        //SET_STEPTICKER_DEBUG_PIN(0);

        // all moves finished
        current_tick = 0;

        // get next block
        // do it here so there is no delay in ticks
        THECONVEYOR->block_finished();

        if(THECONVEYOR->get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue

        }else{
            current_block= nullptr;
            running= false;
        }

        // all moves finished
        // we delegate the slow stuff to the pendsv handler which will run as soon as this interrupt exits
        //NVIC_SetPendingIRQ(PendSV_IRQn); this doesn't work
        //SCB->ICSR = 0x10000000; // SCB_ICSR_PENDSVSET_Msk;
    }
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    // need to prepare each active motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue;

        ok= true; // mark at least one motor is moving
        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction(current_block->direction_bits[m]);
        motor[m]->start_moving(); // also let motor know it is moving now
    }

    if(ok) {
        //SET_STEPTICKER_DEBUG_PIN(1);
        return true;

    }else{
        // this is an edge condition that should never happen, but we need to discard this block if it ever does
        // basically it is a block that has zero steps for all motors
        THECONVEYOR->block_finished();
    }

    return false;
}


// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
