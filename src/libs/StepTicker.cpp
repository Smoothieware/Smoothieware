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

#include "system_LPC17xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
#include "gpio.h"
extern GPIO stepticker_debug_pin;
#endif

StepTicker *StepTicker::instance;

// handle 2.30 Fixed point
#define FPSCALE (1<<30)
#define TOFP(x) ((int32_t)roundf((float)(x)*FPSCALE))
#define FROMFP(x) ((float)(x)/FPSCALE)

StepTicker::StepTicker()
{
    instance = this; // setup the Singleton instance of the stepticker

    // Configure the timer
    LPC_TIM0->MR0 = 10000000;       // Initial dummy value for Match Register
    LPC_TIM0->MCR = 3;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 0;              // Disable interrupt

    LPC_SC->PCONP |= (1 << 2);      // Power Ticker ON
    LPC_TIM1->MR0 = 1000000;
    LPC_TIM1->MCR = 1;
    LPC_TIM1->TCR = 0;              // Disable interrupt

    // Default start values
    this->set_frequency(100000);
    this->set_unstep_time(100);

    this->unstep.reset();
    this->num_motors = 0;

    this->move_issued = false;
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

// Set the reset delay
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf((SystemCoreClock / 4.0F) * (microseconds / 1000000.0F)); // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM1->MR0 = delay;
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

    if(!move_issued){
        if(jobq.empty()) return; // if nothing has been setup we ignore the ticks
        if(!pop_next_job()) return;
    }

    bool still_moving = false;

    // foreach motor, if it is active see if time to issue a step to that motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_job.tick_info[m].steps_to_move == 0) continue; // not active

        still_moving = true;
        current_job.tick_info[m].steps_per_tick += current_job.tick_info[m].acceleration_change;

        if(current_tick == current_job.tick_info[m].next_accel_event) {
            if(current_tick == current_job.block_info.accelerate_until) { // We are done accelerating, deceleration becomes 0 : plateau
                current_job.tick_info[m].acceleration_change = 0;
                if(current_job.block_info.decelerate_after < current_job.block_info.total_move_ticks) {
                    current_job.tick_info[m].next_accel_event = current_job.block_info.decelerate_after;
                    if(current_tick != current_job.block_info.decelerate_after) { // We are plateauing
                        // steps/sec / tick frequency to get steps per tick
                        current_job.tick_info[m].steps_per_tick = current_job.tick_info[m].plateau_rate;
                    }
                }
            }

            if(current_tick == current_job.block_info.decelerate_after) { // We start decelerating
                current_job.tick_info[m].acceleration_change = current_job.tick_info[m].deceleration_change;
            }
        }

        // protect against rounding errors and such
        if(current_job.tick_info[m].steps_per_tick <= 0) {
            current_job.tick_info[m].counter = FPSCALE; // we force completion this step by setting to 1.0
            current_job.tick_info[m].steps_per_tick = 0;
        }

        current_job.tick_info[m].counter += current_job.tick_info[m].steps_per_tick;

        if(current_job.tick_info[m].counter >= FPSCALE) { // >= 1.0 step time
            current_job.tick_info[m].counter -= FPSCALE; // -= 1.0F;
            ++current_job.tick_info[m].step_count;

            // step the motor
            motor[m]->step();
            // we stepped so schedule an unstep
            unstep.set(m);

            if(current_job.tick_info[m].step_count == current_job.tick_info[m].steps_to_move) {
                // done
                current_job.tick_info[m].steps_to_move = 0;
                motor[m]->moving= false; // let motor know it is no longer moving
            }
        }
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

    if(!still_moving) {
        stepticker_debug_pin = 0;

        // all moves finished
        current_tick = 0;

        // get next job
        // do it here so there is no delay in ticks
        // get next job, and copy data
        move_issued= pop_next_job(); // returns false if no new job available

        // all moves finished
        // we delegate the slow stuff to the pendsv handler which will run as soon as this interrupt exits
        //NVIC_SetPendingIRQ(PendSV_IRQn); this doesn't work
        SCB->ICSR = 0x10000000; // SCB_ICSR_PENDSVSET_Msk;
    }
}

// pop next job off queue and copy it for faster access
// only called from the step tick ISR (single consumer)
bool StepTicker::pop_next_job()
{
    // pop next job, return false if nothing to get
    if(!jobq.get(current_job)) return false;

    // need to prepare each active motor
    move_issued = false;
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_job.tick_info[m].steps_to_move == 0) continue;

        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction(current_job.block_info.direction_bits[m]);
        motor[m]->moving= true; // also let motor know it is moving now
        move_issued= true; // set so long as at least one motor is moving
    }

    if(move_issued) stepticker_debug_pin = 1;

    return move_issued;
}

// prepare block for the job queue and push it
// only called from main_loop() (single producer)
// this is done ahead of time so does not delay tick generation, see Conveyor::main_loop()
bool StepTicker::push_block(const Block *block)
{
    //stepticker_debug_pin = 1;
    job_entry_t job;

    job.block_info.accelerate_until = block->accelerate_until;
    job.block_info.decelerate_after = block->decelerate_after;
    job.block_info.total_move_ticks = block->total_move_ticks;
    job.block_info.direction_bits = block->direction_bits;

    float inv = 1.0F / block->steps_event_count;
    for (uint8_t m = 0; m < num_motors; m++) {
        uint32_t steps = block->steps[m];
        job.tick_info[m].steps_to_move = steps;
        if(steps == 0) continue;

        float aratio = inv * steps;
        job.tick_info[m].steps_per_tick = TOFP((block->initial_rate * aratio) / frequency); // steps/sec / tick frequency to get steps per tick in 2.30 fixed point
        job.tick_info[m].counter = 0; // 2.30 fixed point
        job.tick_info[m].step_count = 0;
        job.tick_info[m].next_accel_event = block->total_move_ticks + 1;

        float acceleration_change = 0;
        if(block->accelerate_until != 0) { // If the next accel event is the end of accel
            job.tick_info[m].next_accel_event = block->accelerate_until;
            acceleration_change = block->acceleration_per_tick;

        } else if(block->decelerate_after == 0 /*&& block->accelerate_until == 0*/) {
            // we start off decelerating
            acceleration_change = -block->deceleration_per_tick;

        } else if(block->decelerate_after != block->total_move_ticks /*&& block->accelerate_until == 0*/) {
            // If the next event is the start of decel ( don't set this if the next accel event is accel end )
            job.tick_info[m].next_accel_event = block->decelerate_after;
        }

        // convert to fixed point after scaling
        job.tick_info[m].acceleration_change= TOFP(acceleration_change * aratio);
        job.tick_info[m].deceleration_change= -TOFP(block->deceleration_per_tick * aratio);
        job.tick_info[m].plateau_rate= TOFP((block->maximum_rate * aratio) / frequency);
    }

    //stepticker_debug_pin = 0;
    return jobq.put(job);
}

// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
