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

// A StepperMotor represents an actual stepper motor. It is used to generate steps that move the actual motor at a given speed
// TODO : Abstract this into Actuator

StepperMotor::StepperMotor(){
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->stepped = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    this->remove_from_active_list_next_reset = false;
    this->is_move_finished = false;
    this->signal_step = false;
    this->step_signal_hook = new Hook();

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
}

StepperMotor::StepperMotor(Pin& step, Pin& dir, Pin& en) : step_pin(step), dir_pin(dir), en_pin(en) {
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->stepped = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    this->remove_from_active_list_next_reset = false;
    this->is_move_finished = false;
    this->signal_step = false;
    this->step_signal_hook = new Hook();

    enable(false);
    set_high_on_debug(en.port_number, en.pin);

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
}

// This is called ( see the .h file, we had to put a part of things there for obscure inline reasons ) when a step has to be generated
// we also here check if the move is finished etc ...
void StepperMotor::step(){

    // output to pins 37t
    this->step_pin.set( 1                   );
    this->step_ticker->reset_step_pins = true;

    // move counter back 11t
    this->fx_counter -= this->fx_ticks_per_step;

    // we have moved a step 9t
    this->stepped++;

    // Do we need to signal this step
    if( this->stepped == this->signal_step_number && this->signal_step ){
        this->step_signal_hook->call();
    }

    // Is this move finished ?
    if( this->stepped == this->steps_to_move ){
        // Mark it as finished, then StepTicker will call signal_mode_finished()
        // This is so we don't call that before all the steps have been generated for this tick()
        this->is_move_finished = true;
        this->step_ticker->moves_finished = true;
    }

}


// If the move is finished, the StepTicker will call this ( because we asked it to in tick() )
void StepperMotor::signal_move_finished(){

            // work is done ! 8t
            this->moving = false;
            this->steps_to_move = 0;

            // signal it to whatever cares 41t 411t
            this->end_hook->call();

            // We only need to do this if we were not instructed to move
            if( this->moving == false ){
                this->update_exit_tick();
            }

            this->is_move_finished = false;
}

// This is just a way not to check for ( !this->moving || this->paused || this->fx_ticks_per_step == 0 ) at every tick()
inline void StepperMotor::update_exit_tick(){
    if( !this->moving || this->paused || this->steps_to_move == 0 ){
        // We must exit tick() after setting the pins, no bresenham is done
        //this->remove_from_active_list_next_reset = true;
        this->step_ticker->remove_motor_from_active_list(this);
    }else{
        // We must do the bresenham in tick()
        // We have to do this or there could be a bug where the removal still happens when it doesn't need to
        this->step_ticker->add_motor_to_active_list(this);
    }
}



// Instruct the StepperMotor to move a certain number of steps
void StepperMotor::move( bool direction, unsigned int steps ){
    // We do not set the direction directly, we will set the pin just before the step pin on the next tick
    this->dir_pin.set(direction);
    this->direction = direction;

    // How many steps we have to move until the move is done
    this->steps_to_move = steps;

    // Zero our tool counters
    this->fx_counter = 0;      // Bresenheim counter
    this->stepped = 0;

    // Do not signal steps until we get instructed to
    this->signal_step = false;

    // Starting now we are moving
    if( steps > 0 ){
        this->moving = true;
    }else{
        this->moving = false;
    }
    this->update_exit_tick();

}

// Set the speed at which this steper moves
void StepperMotor::set_speed( float speed ){

    if (speed < 20.0)
        speed = 20.0;

    // How many steps we must output per second
    this->steps_per_second = speed;

    // How many ticks ( base steps ) between each actual step at this speed, in fixed point 64
    float ticks_per_step = (float)( (float)this->step_ticker->frequency / speed );
    float double_fx_ticks_per_step = (float)(1<<8) * ( (float)(1<<8) * ticks_per_step ); // 8x8 because we had to do 16x16 because 32 did not work
    this->fx_ticks_per_step = (uint32_t)( floor(double_fx_ticks_per_step) );

}

// Pause this stepper motor
void StepperMotor::pause(){
    this->paused = true;
    this->update_exit_tick();
}

// Unpause this stepper motor
void StepperMotor::unpause(){
    this->paused = false;
    this->update_exit_tick();
}


void StepperMotor::change_steps_per_mm(float new_steps)
{
    steps_per_mm = new_steps;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
}

void StepperMotor::change_last_milestone(float new_milestone)
{
    last_milestone_mm = new_milestone;
    last_milestone_steps = lround(last_milestone_mm * steps_per_mm);
}

int  StepperMotor::steps_to_target(float target)
{
    int target_steps = lround(target * steps_per_mm);
    return target_steps - last_milestone_steps;
}
