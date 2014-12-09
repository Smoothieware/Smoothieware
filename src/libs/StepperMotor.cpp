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
}

void StepperMotor::init()
{
    // register this motor with the step ticker, and get its index in that array and bit position
    this->index= THEKERNEL->step_ticker->register_motor(this);
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->fx_ticks_per_step = 0xFFFFF000UL; // some big number so we don't start stepping before it is set
    this->stepped = 0;
    this->steps_to_move = 0;
    this->is_move_finished = false;
    this->last_step_tick_valid= false;
    this->last_step_tick= 0;

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;
    minimum_step_rate    = default_minimum_actuator_rate;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
    current_position_steps= 0;
    signal_step= 0;
}


// This is called ( see the .h file, we had to put a part of things there for obscure inline reasons ) when a step has to be generated
// we also here check if the move is finished etc ..
// This is in highest priority interrupt so cannot be pre-empted
void StepperMotor::step()
{
    // ignore if we are still processing the end of a block
    if(this->is_move_finished) return;

    // output to pins 37t
    this->step_pin.set( 1 );

    // move counter back 11t
    this->fx_counter -= this->fx_ticks_per_step;

    // we have moved a step 9t
    this->stepped++;

    // keep track of actuators actual position in steps
    this->current_position_steps += (this->direction ? -1 : 1);

    // we may need to callback on a specific step, usually used to synchronize deceleration timer
    if(this->signal_step != 0 && this->stepped == this->signal_step) {
        THEKERNEL->step_ticker->synchronize_acceleration(true);
        this->signal_step= 0;
    }

    // Is this move finished ?
    if( this->stepped == this->steps_to_move ) {
        // Mark it as finished, then StepTicker will call signal_mode_finished()
        // This is so we don't call that before all the steps have been generated for this tick()
        this->is_move_finished = true;
        THEKERNEL->step_ticker->a_move_finished= true;
        this->last_step_tick= THEKERNEL->step_ticker->get_tick_cnt(); // remember when last step was
    }
}


// If the move is finished, the StepTicker will call this ( because we asked it to in tick() )
void StepperMotor::signal_move_finished()
{
    // work is done ! 8t
    this->moving = false;
    this->steps_to_move = 0;
    this->minimum_step_rate = default_minimum_actuator_rate;

    // signal it to whatever cares
    // in this call a new block may start, new moves set and new speeds
    this->end_hook->call();

    // We only need to do this if we were not instructed to move
    if( !this->moving ) {
        this->update_exit_tick();
    }

    this->is_move_finished = false;
}

// This is just a way not to check for ( !this->moving || this->paused || this->fx_ticks_per_step == 0 ) at every tick()
void StepperMotor::update_exit_tick()
{
    if( !this->moving || this->paused || this->steps_to_move == 0 ) {
        // No more ticks will be recieved and no more events from StepTicker
        THEKERNEL->step_ticker->remove_motor_from_active_list(this);
    } else {
        // we will now get ticks and StepTIcker will send us events
        THEKERNEL->step_ticker->add_motor_to_active_list(this);
    }
}

// Instruct the StepperMotor to move a certain number of steps
StepperMotor* StepperMotor::move( bool direction, unsigned int steps, float initial_speed)
{
    this->dir_pin.set(direction);
    this->direction = direction;

    // How many steps we have to move until the move is done
    this->steps_to_move = steps;

    // Zero our tool counters
    this->stepped = 0;
    this->fx_ticks_per_step = 0xFFFFF000UL; // some big number so we don't start stepping before it is set again
    if(this->last_step_tick_valid) {
        // we set this based on when the last step was, thus compensating for missed ticks
        uint32_t ts= THEKERNEL->step_ticker->ticks_since(this->last_step_tick);
        // if an axis stops too soon then we can get a huge number of ticks here which causes problems, so if the number of ticks is too great we ignore them
        // example of when this happens is when one axis is going very slow an the min 20steps/sec kicks in, the axis will reach its target much sooner leaving a long gap
        // until the end of the block.
        // TODO we may need to set this based on the current step rate, trouble is we don't know what that is yet, we could use the last fx_ticks_per_step as a guide
        if(ts > 5) ts= 5; // limit to 50us catch up around 1-2 steps
        else if(ts > 15) ts= 0; // no way to know what the delay was
        this->fx_counter= ts*fx_increment;
    }else{
        this->fx_counter = 0; // set to zero as there was no step last block
    }

    // Starting now we are moving
    if( steps > 0 ) {
        if(initial_speed >= 0.0F) set_speed(initial_speed);
        this->moving = true;
    } else {
        this->moving = false;
    }
    this->update_exit_tick();
    return this;
}

// Set the speed at which this stepper moves in steps/sec, should be called set_step_rate()
// we need to make sure that we have a minimum speed here and that it fits the 32bit fixed point fx counters
// Note nothing will really ever go as slow as the minimum speed here, it is just forced to avoid bad errors
// fx_ticks_per_step is what actually sets the step rate, it is fixed point 18.14
StepperMotor* StepperMotor::set_speed( float speed )
{
    if(speed < minimum_step_rate) {
        speed= minimum_step_rate;
    }

    // if(speed <= 0.0F) { // we can't actually do 0 but we can get close, need to avoid divide by zero later on
    //     this->fx_ticks_per_step= 0xFFFFFFFFUL; // 0.381 steps/sec
    //     this->steps_per_second = THEKERNEL->step_ticker->get_frequency() / (this->fx_ticks_per_step >> fx_shift);
    //     return;
    // }

    // How many steps we must output per second
    this->steps_per_second = speed;

    // set the new speed, NOTE this can be pre-empted by stepticker so the following write needs to be atomic
    this->fx_ticks_per_step= floor(fx_increment * THEKERNEL->step_ticker->get_frequency() / speed);
    return this;
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
