/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Stepper.h"

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Planner.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "Robot.h"
#include "checksumm.h"
#include "SlowTicker.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"

#include <vector>
using namespace std;

#include "libs/nuts_bolts.h"
#include "libs/Hook.h"

#include <mri.h>


// The stepper reacts to blocks that have XYZ movement to transform them into actual stepper motor moves
// TODO: This does accel, accel should be in StepperMotor

Stepper* stepper;
uint32_t previous_step_count;
uint32_t skipped_speed_updates;
uint32_t speed_ticks_counter;

Stepper::Stepper(){
    this->current_block = NULL;
    this->paused = false;
    this->trapezoid_generator_busy = false;
    this->force_speed_update = false;
    skipped_speed_updates = 0;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded(){
    stepper = this;
    register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);

    // Get onfiguration
    this->on_config_reload(this);

    // Acceleration ticker
    this->acceleration_tick_hook = THEKERNEL->slow_ticker->attach( this->acceleration_ticks_per_second, this, &Stepper::trapezoid_generator_tick );

    // Attach to the end_of_move stepper event
    THEKERNEL->robot->alpha_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move );
    THEKERNEL->robot->beta_stepper_motor->attach( this, &Stepper::stepper_motor_finished_move );
    THEKERNEL->robot->gamma_stepper_motor->attach(this, &Stepper::stepper_motor_finished_move );
}

// Get configuration from the config file
void Stepper::on_config_reload(void* argument){

    this->acceleration_ticks_per_second =  THEKERNEL->config->value(acceleration_ticks_per_second_checksum)->by_default(100   )->as_number();
    this->minimum_steps_per_second      =  THEKERNEL->config->value(minimum_steps_per_minute_checksum     )->by_default(3000  )->as_number() / 60.0F;

    // Steppers start off by default
    this->turn_enable_pins_off();
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Stepper::on_pause(void* argument){
    this->paused = true;
    THEKERNEL->robot->alpha_stepper_motor->pause();
    THEKERNEL->robot->beta_stepper_motor->pause();
    THEKERNEL->robot->gamma_stepper_motor->pause();
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Stepper::on_play(void* argument){
    // TODO: Re-compute the whole queue for a cold-start
    this->paused = false;
    THEKERNEL->robot->alpha_stepper_motor->unpause();
    THEKERNEL->robot->beta_stepper_motor->unpause();
    THEKERNEL->robot->gamma_stepper_motor->unpause();
}

void Stepper::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    // Attach gcodes to the last block for on_gcode_execute
    if( gcode->has_m && (gcode->m == 84 || gcode->m == 17 || gcode->m == 18 )) {
        THEKERNEL->conveyor->append_gcode(gcode);
    }
}

// React to enable/disable gcodes
void Stepper::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    if( gcode->has_m){
        if( gcode->m == 17 ){
            this->turn_enable_pins_on();
        }
        if( (gcode->m == 84 || gcode->m == 18) && !gcode->has_letter('E') ){
            this->turn_enable_pins_off();
        }
    }
}

// Enable steppers
void Stepper::turn_enable_pins_on(){
    for (StepperMotor* m : THEKERNEL->robot->actuators)
        m->enable(true);
    this->enable_pins_status = true;
}

// Disable steppers
void Stepper::turn_enable_pins_off(){
    for (StepperMotor* m : THEKERNEL->robot->actuators)
        m->enable(false);
    this->enable_pins_status = false;
}

// A new block is popped from the queue
void Stepper::on_block_begin(void* argument){
    Block* block  = static_cast<Block*>(argument);

    // The stepper does not care about 0-blocks
    if( block->millimeters == 0.0F ){ return; }

    // Mark the new block as of interrest to us
    if( block->steps[ALPHA_STEPPER] > 0 || block->steps[BETA_STEPPER] > 0 || block->steps[GAMMA_STEPPER] > 0 ){
        block->take();
    }else{
        return;
    }

    // We can't move with the enable pins off
    if( this->enable_pins_status == false ){
        this->turn_enable_pins_on();
    }

    // Setup : instruct stepper motors to move
    if( block->steps[ALPHA_STEPPER] > 0 ){ THEKERNEL->robot->alpha_stepper_motor->move( ( block->direction_bits >> 0  ) & 1 , block->steps[ALPHA_STEPPER] ); }
    if( block->steps[BETA_STEPPER ] > 0 ){ THEKERNEL->robot->beta_stepper_motor->move(  ( block->direction_bits >> 1  ) & 1 , block->steps[BETA_STEPPER ] ); }
    if( block->steps[GAMMA_STEPPER] > 0 ){ THEKERNEL->robot->gamma_stepper_motor->move( ( block->direction_bits >> 2  ) & 1 , block->steps[GAMMA_STEPPER] ); }

    this->current_block = block;

    // Setup acceleration for this block
    this->trapezoid_generator_reset();

    // Find the stepper with the more steps, it's the one the speed calculations will want to follow
    this->main_stepper = THEKERNEL->robot->alpha_stepper_motor;
    if( THEKERNEL->robot->beta_stepper_motor->steps_to_move > this->main_stepper->steps_to_move ){ this->main_stepper = THEKERNEL->robot->beta_stepper_motor; }
    if( THEKERNEL->robot->gamma_stepper_motor->steps_to_move > this->main_stepper->steps_to_move ){ this->main_stepper = THEKERNEL->robot->gamma_stepper_motor; }

    // Set the initial speed for this move
    this->trapezoid_generator_tick(0);

    // Synchronise the acceleration curve with the stepping
    this->synchronize_acceleration(0);

}

// Current block is discarded
void Stepper::on_block_end(void* argument){
    this->current_block = NULL; //stfu !
}

// When a stepper motor has finished it's assigned movement
uint32_t Stepper::stepper_motor_finished_move(uint32_t dummy){

    // We care only if none is still moving
    if( THEKERNEL->robot->alpha_stepper_motor->moving || THEKERNEL->robot->beta_stepper_motor->moving || THEKERNEL->robot->gamma_stepper_motor->moving ){ return 0; }

    // This block is finished, release it
    if( this->current_block != NULL ){
        this->current_block->release();
    }

    return 0;
}


// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
uint32_t Stepper::trapezoid_generator_tick( uint32_t dummy ) {

    // Do not do the accel math for nothing
    if(this->current_block && !this->paused && this->main_stepper->moving ) {

        // Store this here because we use it a lot down there
        uint32_t current_steps_completed = this->main_stepper->stepped;

        // Do not accel, just set the value
        if( this->force_speed_update ){
          this->force_speed_update = false;
          this->set_step_events_per_second(this->trapezoid_adjusted_rate);
          return 0;
        }

        // If we are accelerating
        if(current_steps_completed <= this->current_block->accelerate_until + 1) {
            // Increase speed
            this->trapezoid_adjusted_rate += this->current_block->rate_delta;
              if (this->trapezoid_adjusted_rate > this->current_block->nominal_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
              }
              this->set_step_events_per_second(this->trapezoid_adjusted_rate);

        // If we are decelerating
        }else if (current_steps_completed > this->current_block->decelerate_after) {
             // Reduce speed
             // NOTE: We will only reduce speed if the result will be > 0. This catches small
              // rounding errors that might leave steps hanging after the last trapezoid tick.
              if(this->trapezoid_adjusted_rate > this->current_block->rate_delta * 1.5F) {
                  this->trapezoid_adjusted_rate -= this->current_block->rate_delta;
              }else{
                  this->trapezoid_adjusted_rate = this->current_block->rate_delta * 1.5F;
              }
              if(this->trapezoid_adjusted_rate < this->current_block->final_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->final_rate;
              }
              this->set_step_events_per_second(this->trapezoid_adjusted_rate);

        // If we are cruising
        }else {
              // Make sure we cruise at exactly nominal rate
              if (this->trapezoid_adjusted_rate != this->current_block->nominal_rate) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
                  this->set_step_events_per_second(this->trapezoid_adjusted_rate);
              }
          }
    }

    return 0;
}



// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
inline void Stepper::trapezoid_generator_reset(){
    this->trapezoid_adjusted_rate = this->current_block->initial_rate;
    this->force_speed_update = true;
    this->trapezoid_tick_cycle_counter = 0;
    previous_step_count = 0;
    skipped_speed_updates = 0;
    speed_ticks_counter = 0;
}

// Update the speed for all steppers
void Stepper::set_step_events_per_second( float steps_per_second )
{
    // We do not step slower than this
    //steps_per_second = max(steps_per_second, this->minimum_steps_per_second);
    if( steps_per_second < this->minimum_steps_per_second ){
        steps_per_second = this->minimum_steps_per_second;
    }

    // Instruct the stepper motors
    if( THEKERNEL->robot->alpha_stepper_motor->moving ){ THEKERNEL->robot->alpha_stepper_motor->set_speed( steps_per_second * ( (float)this->current_block->steps[ALPHA_STEPPER] / (float)this->current_block->steps_event_count ) ); }
    if( THEKERNEL->robot->beta_stepper_motor->moving  ){ THEKERNEL->robot->beta_stepper_motor->set_speed(  steps_per_second * ( (float)this->current_block->steps[BETA_STEPPER ] / (float)this->current_block->steps_event_count ) ); }
    if( THEKERNEL->robot->gamma_stepper_motor->moving ){ THEKERNEL->robot->gamma_stepper_motor->set_speed( steps_per_second * ( (float)this->current_block->steps[GAMMA_STEPPER] / (float)this->current_block->steps_event_count ) ); }

    // Other modules might want to know the speed changed
    THEKERNEL->call_event(ON_SPEED_CHANGE, this);

}

// This function has the role of making sure acceleration and deceleration curves have their
// rhythm synchronized. The accel/decel must start at the same moment as the speed update routine
// This is caller in "step just occured" or "block just began" ( step Timer ) context, so we need to be fast.
// All we do is reset the other timer so that it does what we want
uint32_t Stepper::synchronize_acceleration(uint32_t dummy){

    // No move was done, this is called from on_block_begin
    // This means we setup the accel timer in a way where it gets called right after
    // we exit this step interrupt, and so that it is then in synch with
    if( this->main_stepper->stepped == 0 ){
        // Whatever happens, we must call the accel interrupt asap
        // Because it will set the initial rate
        // We also want to synchronize in case we start accelerating or decelerating now

        // Accel interrupt must happen asap
        NVIC_SetPendingIRQ(TIMER2_IRQn);
        // Synchronize both counters
        LPC_TIM2->TC = LPC_TIM0->TC;

        // If we start decelerating after this, we must ask the actuator to warn us
        // so we can do what we do in the "else" bellow
        if( this->current_block->decelerate_after > 0 && this->current_block->decelerate_after < this->main_stepper->steps_to_move ){
            this->main_stepper->attach_signal_step(this->current_block->decelerate_after, this, &Stepper::synchronize_acceleration);
        }
    }else{
        // If we are called not at the first steps, this means we are beginning deceleration
        NVIC_SetPendingIRQ(TIMER2_IRQn);
        // Synchronize both counters
        LPC_TIM2->TC = LPC_TIM0->TC;
    }

    return 0;
}

