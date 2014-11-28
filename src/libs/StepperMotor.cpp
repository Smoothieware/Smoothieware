/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>

// in steps/sec the default minimum speed (was 20steps/sec hardcoded)
float StepperMotor::default_minimum_actuator_rate= 20.0F;

// A StepperMotor represents an actual stepper motor. It is used to generate steps that move the actual motor at a given speed
// TODO : Abstract this into Actuator

StepperMotor::StepperMotor()
{
    init();
}

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    init();
    enable(false);
    set_high_on_debug(en.port_number, en.pin);
}

StepperMotor::~StepperMotor()
{
    delete step_signal_hook;
}

void StepperMotor::init()
{
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->stepped = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    this->is_move_finished = false;
    this->signal_step = false;
    this->step_signal_hook = new Hook();

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;
    minimum_step_rate    = default_minimum_actuator_rate;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
    current_position_steps= 0;
}


// This is called ( see the .h file, we had to put a part of things there for obscure inline reasons ) when a step has to be generated
// we also here check if the move is finished etc ...
void StepperMotor::step()
{
    if(this->is_move_finished) return; // we can't do anything until the next move has been processed, but we will be able to offset the time by shortening the next step

    // output to pins 37t
    this->step_pin.set( 1 );
    THEKERNEL->step_ticker->reset_step_pins = true;

    // move counter back 11t
    if(this->fx_counter > this->fx_ticks_per_step) {
        this->fx_counter -= this->fx_ticks_per_step;
    }else{
        this->fx_counter= 0; // can't make it less than 0 as it is a uint, unless we get interrupted I don't think this case is possible
    }

    // we have moved a step 9t
    this->stepped++;

    // Do we need to signal this step
    if( this->stepped == this->signal_step_number && this->signal_step ) {
        this->step_signal_hook->call();
    }

    // keep track of actuators actual position in steps
    this->current_position_steps += (this->direction ? -1 : 1);

    // Is this move finished ?
    if( this->stepped == this->steps_to_move ) {
        // Mark it as finished, then StepTicker will call signal_mode_finished()
        // This is so we don't call that before all the steps have been generated for this tick()
        this->is_move_finished = true;
        THEKERNEL->step_ticker->a_move_finished = true;
        this->fx_counter = 0;      // set this to zero here so we don't miss any for next move
    }
}


// If the move is finished, the StepTicker will call this ( because we asked it to in tick() )
void StepperMotor::signal_move_finished()
{
    // work is done ! 8t
    this->moving = false;
    this->steps_to_move = 0;
    this->minimum_step_rate = default_minimum_actuator_rate;

    // signal it to whatever cares 41t 411t
    this->end_hook->call();

    // We only need to do this if we were not instructed to move
    if( this->moving == false ) {
        this->update_exit_tick();
    }

    this->is_move_finished = false;
}

// This is just a way not to check for ( !this->moving || this->paused || this->fx_ticks_per_step == 0 ) at every tick()
inline void StepperMotor::update_exit_tick()
{
    if( !this->moving || this->paused || this->steps_to_move == 0 ) {
        // We must exit tick() after setting the pins, no bresenham is done
        THEKERNEL->step_ticker->remove_motor_from_active_list(this);
    } else {
        // We must do the bresenham in tick()
        // We have to do this or there could be a bug where the removal still happens when it doesn't need to
        THEKERNEL->step_ticker->add_motor_to_active_list(this);
    }
}

// Instruct the StepperMotor to move a certain number of steps
void StepperMotor::move( bool direction, unsigned int steps, float initial_speed)
{
    this->dir_pin.set(direction);
    this->direction = direction;

    // How many steps we have to move until the move is done
    this->steps_to_move = steps;

    // Zero our tool counters
    this->stepped = 0;

    // Do not signal steps until we get instructed to
    this->signal_step = false;

    // Starting now we are moving
    if( steps > 0 ) {
        if(initial_speed >= 0.0F) set_speed(initial_speed);
        this->moving = true;
    } else {
        this->moving = false;
    }
    this->update_exit_tick();
}

// this is called to set the step rate based on this blocks rate, we use this instead of set_speed for coordinated moves
// so that we can floor to a minimum speed which is proportional for all axis. the minimum step rate is set at the start
// of each block based on the slowest axis of all coordinated axis.
// the rate passed in is the requested rate, it is scaled for this motor based on steps_to_move and block_steps_event_count
void StepperMotor::set_step_rate(float requested_rate, uint32_t block_steps_event_count)
{
    float rate= requested_rate * ((float)steps_to_move / (float)block_steps_event_count);
    if(rate < minimum_step_rate) {
        rate= minimum_step_rate;
    }
    set_speed(rate);
}

// Set the speed at which this stepper moves in steps/sec, should be called set_step_rate()
// we need to make sure that we have a minimum speed here and that it fits the 64bit fixed point fx counters
// Note nothing will really ever go as slow as the minimum speed here, it is just forced to avoid bad errors
// fx_ticks_per_step is what actually sets the step rate, it is fixed point 32.32
void StepperMotor::set_speed( float speed )
{
    if(speed <= 0.0F) { // we can't actually do 0 but we can get close, need to avoid divide by zero later on
        this->fx_ticks_per_step= 0xFFFFF00000000000ULL; // that is 4,294,963,200 10us ticks which is ~11.9 hours for 1 step
        this->steps_per_second = THEKERNEL->step_ticker->frequency / (this->fx_ticks_per_step>>fx_shift);
        return;
    }

    // How many steps we must output per second
    this->steps_per_second = speed;

    // How many ticks ( base steps ) between each actual step at this speed, in fixed point 64
    // we need to use double here to match the 64bit resolution of the ticker
    double ticks_per_step = (double)THEKERNEL->step_ticker->frequency / speed;
    if(ticks_per_step > 0xFFFFF000UL) { // maximum we can really do and allow a few overflow steps
        ticks_per_step= 0xFFFFF000UL;
        this->steps_per_second = THEKERNEL->step_ticker->frequency / ticks_per_step;
    }
    double double_fx_ticks_per_step = fx_increment * ticks_per_step;

    // set the new speed
    this->fx_ticks_per_step = floor(double_fx_ticks_per_step);
}

// Pause this stepper motor
void StepperMotor::pause()
{
    this->paused = true;
    this->update_exit_tick();
}

// Unpause this stepper motor
void StepperMotor::unpause()
{
    this->paused = false;
    this->update_exit_tick();
}


void StepperMotor::change_steps_per_mm(float new_steps)
{
    steps_per_mm = new_steps;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

void StepperMotor::change_last_milestone(float new_milestone)
{
    last_milestone_mm = new_milestone;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

int  StepperMotor::steps_to_target(float target)
{
    int target_steps = lround(target * steps_per_mm);
    return target_steps - last_milestone_steps;
}
