/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#include "mri.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"

StepperMotor::StepperMotor(){
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    this->direction_bit = 0;
    this->step_bit = 0;
    this->update_exit_tick();
}

StepperMotor::StepperMotor(Pin* step, Pin* dir, Pin* en) : step_pin(step), dir_pin(dir), en_pin(en) {
    this->moving = false;
    this->paused = false;
    this->fx_counter = 0;
    this->fx_ticks_per_step = 0;
    this->steps_to_move = 0;
    this->direction_bit = 0;
    this->step_bit = 0;
    this->update_exit_tick();
}

// Called a great many times per second, to step if we have to now
bool StepperMotor::tick(){

    // output to pins 37t
    this->dir_pin->set(  this->direction_bit );
    this->step_pin->set( this->step_bit      );

    // ignore inactive steppers 13t
    if( this->exit_tick ){ 
        this->step_bit = 0; 
        return false; 
    }
    
    // increase the ( fixed point ) counter by one tick 11t
    this->fx_counter += (uint64_t)((uint64_t)1<<32);  

    // if we are to step now 10t
    if( this->fx_counter >= this->fx_ticks_per_step ){
    
        // move counter back 11t
        this->fx_counter -= this->fx_ticks_per_step;

        // we must step, actual output is done at the beginning of this function 8t
        this->step_bit = 1;

        // we have moved a step 9t
        this->stepped++;

        // is this move finished ? 11t
        if( this->stepped == this->steps_to_move ){

            // work is done ! 8t
            this->moving = false;
            this->update_exit_tick(); 

            // signal it to whatever cares 41t 411t
            this->end_hook->call();

        }

    }else{
        
        // we must not step
        this->step_bit = 0;
    }

}

// This is just a way not to check for ( !this->moving || this->paused || this->fx_ticks_per_step == 0 ) at every tick()
inline void StepperMotor::update_exit_tick(){
    if( !this->moving || this->paused || this->fx_ticks_per_step == 0 ){ 
        this->exit_tick = true; 
    }else{
        this->exit_tick = false;
    }
}



// Instruct the StepperMotor to move a certain number of steps
void StepperMotor::move( bool direction, unsigned int steps ){
   
    //printf("stepper move %p moving %u steps\r\n", this, steps);

    // We do not set the direction directly, we will set the pin just before the step pin on the next tick 
    this->direction_bit = direction;

    // How many steps we have to move until the move is done
    this->steps_to_move = steps;

    // Zero our tool counters
    this->fx_counter = 0;      // Bresenheim counter
    this->stepped = 0;

    // Starting now we are moving
    if( steps > 0 ){ this->moving = true; }else{ this->moving = false; }
    this->update_exit_tick(); 

}

// Set the speed at which this steper moves
void StepperMotor::set_speed( double speed ){

    if( speed < 0.0001 ){ 
        this->steps_per_second = 0;
        this->fx_ticks_per_step = 1>>63;
        return; 
    }
    
    // How many steps we must output per second
    this->steps_per_second = speed;

    // How many ticks ( base steps ) between each actual step at this speed, in fixed point 64
    double ticks_per_step = (double)( (double)this->step_ticker->frequency / speed );
    double double_fx_ticks_per_step = (double)(1<<16) * ( (double)(1<<16) * ticks_per_step );
    this->fx_ticks_per_step = (uint64_t)( floor(double_fx_ticks_per_step) );
    this->update_exit_tick(); 

}




