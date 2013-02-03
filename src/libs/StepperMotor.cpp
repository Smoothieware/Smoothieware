/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "mri.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "MRI_Hooks.h"

StepperMotor::StepperMotor(){
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->stepped = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    //this->direction_bit = 0;
    this->remove_from_active_list_next_reset = false;
    this->is_move_finished = false;
    this->signal_step = false;
    this->step_signal_hook = new Hook();
}

StepperMotor::StepperMotor(Pin* step, Pin* dir, Pin* en) : step_pin(step), dir_pin(dir), en_pin(en) {
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->stepped = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    //this->direction_bit = 0;
    this->remove_from_active_list_next_reset = false;
    this->is_move_finished = false;
    this->signal_step = false;
    this->step_signal_hook = new Hook();

    set_high_on_debug(en->port_number, en->pin);
}

// Called a great many times per second, to step if we have to now
void StepperMotor::tick(){

    // increase the ( fixed point ) counter by one tick 11t
    this->fx_counter += (uint64_t)((uint64_t)1<<32);

    // if we are to step now 10t
    if( this->fx_counter >= this->fx_ticks_per_step ){

        // output to pins 37t
        this->step_pin->set( 1                   );
        this->step_ticker->reset_step_pins = true;

        // move counter back 11t
        this->fx_counter -= this->fx_ticks_per_step;

        // we have moved a step 9t
        this->stepped++;

        // Do we need to signal this step
        if( this->stepped == this->signal_step_number && this->signal_step ){
            this->step_signal_hook->call();
        }

        // is this move finished ? 11t
        if( this->stepped == this->steps_to_move ){
            this->is_move_finished = true;
            this->step_ticker->moves_finished = true;
        }

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
    this->dir_pin->set(direction);

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

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// Set the speed at which this steper moves
void StepperMotor::set_speed( double speed ){
//     if( speed <= 1.0 ){
//         this->steps_per_second = 0;
//         this->fx_ticks_per_step = 1ULL<<63;
//         return;
//     }

    //if( speed < this->steps_per_second ){
        LPC_GPIO1->FIOSET = 1<<19;
    //}

    if (speed < 1.0)
        speed = 1.0;

    // How many steps we must output per second
    this->steps_per_second = speed;

    // How many ticks ( base steps ) between each actual step at this speed, in fixed point 64
    double ticks_per_step = (double)( (double)this->step_ticker->frequency / speed );
    double double_fx_ticks_per_step = (double)(1<<16) * ( (double)(1<<16) * ticks_per_step );
    this->fx_ticks_per_step = (uint64_t)( floor(double_fx_ticks_per_step) );

}

//#pragma GCC pop_options

void StepperMotor::pause(){
    this->paused = true;
    this->update_exit_tick();
}

void StepperMotor::unpause(){
    this->paused = false;
    this->update_exit_tick();
}


