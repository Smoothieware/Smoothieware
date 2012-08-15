/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Stepper.h"
#include "Planner.h"
#include "Player.h"
#include <vector>
using namespace std;
#include "libs/nuts_bolts.h"

Stepper* stepper;

Stepper::Stepper(){
    this->current_block = NULL;
    this->paused = false;
    this->trapezoid_generator_busy = false;
    this->force_speed_update = false;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded(){
    stepper = this;
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
 
    // Get onfiguration
    this->on_config_reload(this); 

    // Acceleration ticker
    this->kernel->slow_ticker->attach( this->acceleration_ticks_per_second, this, &Stepper::trapezoid_generator_tick );

    // Attach to the end_of_move stepper event
    this->kernel->robot->alpha_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move ); 
    this->kernel->robot->beta_stepper_motor->attach( this, &Stepper::stepper_motor_finished_move ); 
    this->kernel->robot->gamma_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move ); 

}

// Get configuration from the config file
void Stepper::on_config_reload(void* argument){
    
    this->acceleration_ticks_per_second =  this->kernel->config->value(acceleration_ticks_per_second_checksum)->by_default(100   )->as_number();
    this->minimum_steps_per_minute      =  this->kernel->config->value(minimum_steps_per_minute_checksum     )->by_default(1200  )->as_number();

    // Steppers start on by default
    this->turn_enable_pins_on();

}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Stepper::on_pause(void* argument){
    this->paused = true;
    this->kernel->robot->alpha_stepper_motor->paused = true;
    this->kernel->robot->beta_stepper_motor->paused  = true;
    this->kernel->robot->gamma_stepper_motor->paused = true;

}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Stepper::on_play(void* argument){
    // TODO: Re-compute the whole queue for a cold-start
    this->paused = false;
    this->kernel->robot->alpha_stepper_motor->paused = true;
    this->kernel->robot->beta_stepper_motor->paused  = true;
    this->kernel->robot->gamma_stepper_motor->paused = true;
}


void Stepper::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('M')){
        int code = (int) gcode->get_value('M');
        if( code == 17 ){
            this->turn_enable_pins_on(); 
        }
        if( code == 84 || code == 18 ){
            this->turn_enable_pins_off(); 
        }
    }
}

void Stepper::turn_enable_pins_on(){
    this->kernel->robot->alpha_en_pin->set(0);
    this->kernel->robot->beta_en_pin->set(0);
    this->kernel->robot->gamma_en_pin->set(0);
    this->enable_pins_status = true;
}

void Stepper::turn_enable_pins_off(){
    this->kernel->robot->alpha_en_pin->set(1);
    this->kernel->robot->beta_en_pin->set(1);
    this->kernel->robot->gamma_en_pin->set(1);
    this->enable_pins_status = false;
}

// A new block is popped from the queue
void Stepper::on_block_begin(void* argument){
    Block* block  = static_cast<Block*>(argument);

    //printf("block begin\r\n");

    //printf("%p: steps:%4d|%4d|%4d(max:%4d) nominal:r%10d/s%6.1f mm:%9.6f rdelta:%8d acc:%5d dec:%5d rates:%10d>%10d taken:%d ready:%d \r\n", block, block->steps[0], block->steps[1], block->steps[2], block->steps_event_count, block->nominal_rate, block->nominal_speed, block->millimeters, block->rate_delta, block->accelerate_until, block->decelerate_after, block->initial_rate, block->final_rate, block->times_taken, block->is_ready );
    
    // The stepper does not care about 0-blocks
    if( block->millimeters == 0.0 ){ return; }
  
    // We can't move with the enable pins off 
    if( this->enable_pins_status == false ){
        this->turn_enable_pins_on();
    }

    // Mark the new block as of interrest to us
    block->take();
    
    // Setup : instruct stepper motors to move
    if( block->steps[ALPHA_STEPPER] > 0 ){ this->kernel->robot->alpha_stepper_motor->move( ( block->direction_bits >> 0  ) & 1 , block->steps[ALPHA_STEPPER] ); } 
    if( block->steps[BETA_STEPPER ] > 0 ){ this->kernel->robot->beta_stepper_motor->move(  ( block->direction_bits >> 1  ) & 1 , block->steps[BETA_STEPPER ] ); } 
    if( block->steps[GAMMA_STEPPER] > 0 ){ this->kernel->robot->gamma_stepper_motor->move( ( block->direction_bits >> 2  ) & 1 , block->steps[GAMMA_STEPPER] ); } 

    this->current_block = block;

    // Setup acceleration for this block 
    this->trapezoid_generator_reset();

    // Find the stepper with the more steps, it's the one the speed calculations will want to follow
    this->main_stepper = this->kernel->robot->alpha_stepper_motor;
    if( this->kernel->robot->beta_stepper_motor->steps_to_move > this->main_stepper->steps_to_move ){ this->main_stepper = this->kernel->robot->beta_stepper_motor; }
    if( this->kernel->robot->gamma_stepper_motor->steps_to_move > this->main_stepper->steps_to_move ){ this->main_stepper = this->kernel->robot->gamma_stepper_motor; }


    if( block->initial_rate == 0 ){
        this->trapezoid_generator_tick(0);
    }

}

// Current block is discarded
void Stepper::on_block_end(void* argument){
    Block* block  = static_cast<Block*>(argument);
    this->current_block = NULL; //stfu !
}

// When a stepper motor has finished it's assigned movement
inline uint32_t Stepper::stepper_motor_finished_move(uint32_t dummy){

    // We care only if none is still moving
    if( this->kernel->robot->alpha_stepper_motor->moving || this->kernel->robot->beta_stepper_motor->moving || this->kernel->robot->gamma_stepper_motor->moving ){ return 0; }
    
    // This block is finished, release it
    if( this->current_block != NULL ){
        this->current_block->release(); 
    }

}

inline uint32_t Stepper::step_events_completed(){
    return this->main_stepper->stepped;
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
uint32_t Stepper::trapezoid_generator_tick( uint32_t dummy ) {

    if(this->current_block && !this->paused ) {
        if(this->step_events_completed() < this->current_block->accelerate_until) {
              this->trapezoid_adjusted_rate += this->current_block->rate_delta;
              if (this->trapezoid_adjusted_rate > this->current_block->nominal_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
              }
              this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
          }else if (this->step_events_completed() >= this->current_block->decelerate_after) {
              // NOTE: We will only reduce speed if the result will be > 0. This catches small
              // rounding errors that might leave steps hanging after the last trapezoid tick.
              if(this->trapezoid_adjusted_rate > double(this->current_block->rate_delta) * 1.5) {
                  this->trapezoid_adjusted_rate -= this->current_block->rate_delta;
              }else{
                  this->trapezoid_adjusted_rate = double(this->current_block->rate_delta) * 1.5; 
                  //this->trapezoid_adjusted_rate = floor(double(this->trapezoid_adjusted_rate / 2 ));
              }
              if(this->trapezoid_adjusted_rate < this->current_block->final_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->final_rate;
              }
              this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
          }else {
              // Make sure we cruise at exactly nominal rate
              if (this->trapezoid_adjusted_rate != this->current_block->nominal_rate) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
                  this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
              }
          }
          if( this->force_speed_update ){
              this->force_speed_update = false;
              this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
          }
    }

}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
inline void Stepper::trapezoid_generator_reset(){
    this->trapezoid_adjusted_rate = this->current_block->initial_rate;
    this->force_speed_update = true;
    this->trapezoid_tick_cycle_counter = 0;
}

// Update the speed for all steppers
void Stepper::set_step_events_per_minute( double steps_per_minute ){

    // We do not step slower than this 
    steps_per_minute = max(steps_per_minute, this->minimum_steps_per_minute);

    // Instruct the stepper motors
    if( this->kernel->robot->alpha_stepper_motor->moving ){ this->kernel->robot->alpha_stepper_motor->set_speed( (steps_per_minute/60L) * ( (double)this->current_block->steps[ALPHA_STEPPER] / (double)this->current_block->steps_event_count ) ); }
    if( this->kernel->robot->beta_stepper_motor->moving  ){ this->kernel->robot->beta_stepper_motor->set_speed(  (steps_per_minute/60L) * ( (double)this->current_block->steps[BETA_STEPPER ] / (double)this->current_block->steps_event_count ) ); }
    if( this->kernel->robot->gamma_stepper_motor->moving ){ this->kernel->robot->gamma_stepper_motor->set_speed( (steps_per_minute/60L) * ( (double)this->current_block->steps[GAMMA_STEPPER] / (double)this->current_block->steps_event_count ) ); }

    this->kernel->call_event(ON_SPEED_CHANGE, this);
}

