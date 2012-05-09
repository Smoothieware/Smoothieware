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
    this->step_events_completed = 0; 
    this->divider = 0;
    this->paused = false;
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
    //this->kernel->slow_ticker->set_frequency(this->acceleration_ticks_per_second/10);
    this->kernel->slow_ticker->attach( this->acceleration_ticks_per_second, this, &Stepper::trapezoid_generator_tick );

    // Initiate main_interrupt timer and step reset timer
    this->kernel->step_ticker->attach( this, &Stepper::main_interrupt );   
    this->kernel->step_ticker->reset_attach( this, &Stepper::reset_step_pins );

}

// Get configuration from the config file
void Stepper::on_config_reload(void* argument){
    
    this->microseconds_per_step_pulse   =  this->kernel->config->value(microseconds_per_step_pulse_checksum  )->by_default(5     )->as_number();
    this->acceleration_ticks_per_second =  this->kernel->config->value(acceleration_ticks_per_second_checksum)->by_default(100   )->as_number();
    this->minimum_steps_per_minute      =  this->kernel->config->value(minimum_steps_per_minute_checksum     )->by_default(1200  )->as_number();
    this->base_stepping_frequency       =  this->kernel->config->value(base_stepping_frequency_checksum      )->by_default(100000)->as_number();
    this->alpha_step_pin                =  this->kernel->config->value(alpha_step_pin_checksum               )->by_default("1.21"     )->as_pin()->as_output();
    this->beta_step_pin                 =  this->kernel->config->value(beta_step_pin_checksum                )->by_default("1.23"     )->as_pin()->as_output();
    this->gamma_step_pin                =  this->kernel->config->value(gamma_step_pin_checksum               )->by_default("1.22!"    )->as_pin()->as_output();
    this->alpha_dir_pin                 =  this->kernel->config->value(alpha_dir_pin_checksum                )->by_default("1.18"     )->as_pin()->as_output();
    this->beta_dir_pin                  =  this->kernel->config->value(beta_dir_pin_checksum                 )->by_default("1.20"     )->as_pin()->as_output();
    this->gamma_dir_pin                 =  this->kernel->config->value(gamma_dir_pin_checksum                )->by_default("1.19"     )->as_pin()->as_output();
    this->alpha_en_pin                  =  this->kernel->config->value(alpha_en_pin_checksum                 )->by_default("0.4"      )->as_pin()->as_output()->as_open_drain();
    this->beta_en_pin                   =  this->kernel->config->value(beta_en_pin_checksum                  )->by_default("0.10"     )->as_pin()->as_output()->as_open_drain();
    this->gamma_en_pin                  =  this->kernel->config->value(gamma_en_pin_checksum                 )->by_default("0.19"     )->as_pin()->as_output()->as_open_drain();


    // TODO : This is supposed to be done by gcodes
    this->alpha_en_pin->set(0);
    this->beta_en_pin->set(0);
    this->gamma_en_pin->set(0);


    // Set the Timer interval for Match Register 1, 
    this->kernel->step_ticker->set_reset_delay( this->microseconds_per_step_pulse / 1000000 );
    this->kernel->step_ticker->set_frequency( this->base_stepping_frequency );
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Stepper::on_pause(void* argument){
    this->paused = true;
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Stepper::on_play(void* argument){
    // TODO: Re-compute the whole queue for a cold-start
    this->paused = false;
}

void Stepper::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('M')){
        int code = (int) gcode->get_value('M');
        if( code == 84 ){
            this->alpha_en_pin->set(0);
            this->beta_en_pin->set(0);
            this->gamma_en_pin->set(0);
        }
    }
}

// A new block is popped from the queue
void Stepper::on_block_begin(void* argument){
    Block* block  = static_cast<Block*>(argument);

    // The stepper does not care about 0-blocks
    if( block->millimeters == 0.0 ){ return; }
    
    // Mark the new block as of interrest to us
    block->take();
   
    // Setup
    for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ this->counters[stpr] = 0; this->stepped[stpr] = 0; } 
    this->step_events_completed = 0; 

    this->current_block = block;
    
    // This is just to save computing power and not do it every step 
    this->update_offsets();
    
    // Setup acceleration for this block 
    this->trapezoid_generator_reset();

}

// Current block is discarded
void Stepper::on_block_end(void* argument){
    Block* block  = static_cast<Block*>(argument);
    this->current_block = NULL; //stfu !
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Smoothie. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
inline uint32_t Stepper::main_interrupt(uint32_t dummy){
    if( this->paused ){ return 0; } 

    // Step dir pins first, then step pinse, stepper drivers like to know the direction before the step signal comes in
    this->alpha_dir_pin->set(  ( this->out_bits >> 0  ) & 1 );
    this->beta_dir_pin->set(   ( this->out_bits >> 1  ) & 1 );
    this->gamma_dir_pin->set(  ( this->out_bits >> 2  ) & 1 );
    this->alpha_step_pin->set( ( this->out_bits >> 3  ) & 1 );
    this->beta_step_pin->set(  ( this->out_bits >> 4  ) & 1 );
    this->gamma_step_pin->set( ( this->out_bits >> 5  ) & 1 );

    if( this->current_block != NULL ){
        // Set bits for direction and steps 
        this->out_bits = this->current_block->direction_bits;
        for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ 
            this->counters[stpr] += this->counter_increment; 
            if( this->counters[stpr] > this->offsets[stpr] && this->stepped[stpr] < this->current_block->steps[stpr] ){
                this->counters[stpr] -= this->offsets[stpr] ;
                this->stepped[stpr]++;
                this->out_bits |= (1 << (stpr+3) );
            } 
        } 
        // If current block is finished, reset pointer
        this->step_events_completed += this->counter_increment;
        if( this->step_events_completed >= this->current_block->steps_event_count<<16 ){ 
            if( this->stepped[ALPHA_STEPPER] == this->current_block->steps[ALPHA_STEPPER] && this->stepped[BETA_STEPPER] == this->current_block->steps[BETA_STEPPER] && this->stepped[GAMMA_STEPPER] == this->current_block->steps[GAMMA_STEPPER] ){ 
                if( this->current_block != NULL ){
                    this->current_block->release(); 
                }
            }
        }
    }else{
        this->out_bits = 0;
    }

}

// We compute this here instead of each time in the interrupt
void Stepper::update_offsets(){
    for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ 
        this->offsets[stpr] = (int)floor((float)((float)(1<<16)*(float)((float)this->current_block->steps_event_count / (float)this->current_block->steps[stpr]))); 
    }
}

// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
uint32_t Stepper::trapezoid_generator_tick( uint32_t dummy ) {
    if(this->current_block && !this->trapezoid_generator_busy && !this->paused ) {
          if(this->step_events_completed < this->current_block->accelerate_until<<16) {
              this->trapezoid_adjusted_rate += this->current_block->rate_delta;
              if (this->trapezoid_adjusted_rate > this->current_block->nominal_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
              }
              this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
          } else if (this->step_events_completed >= this->current_block->decelerate_after<<16) {
              // NOTE: We will only reduce speed if the result will be > 0. This catches small
              // rounding errors that might leave steps hanging after the last trapezoid tick.
              if (this->trapezoid_adjusted_rate > double(this->current_block->rate_delta) * 1.5) {
                  this->trapezoid_adjusted_rate -= this->current_block->rate_delta;
              }else{
                  this->trapezoid_adjusted_rate = double(this->current_block->rate_delta) * 1.5; 
                  //this->trapezoid_adjusted_rate = floor(double(this->trapezoid_adjusted_rate / 2 ));
                  //this->kernel->serial->printf("over!\r\n");
              }
              if (this->trapezoid_adjusted_rate < this->current_block->final_rate ) {
                  this->trapezoid_adjusted_rate = this->current_block->final_rate;
              }
              this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
          } else {
              // Make sure we cruise at exactly nominal rate
              if (this->trapezoid_adjusted_rate != this->current_block->nominal_rate) {
                  this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
                  this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
              }
          }
    }
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
void Stepper::trapezoid_generator_reset(){
    this->trapezoid_adjusted_rate = this->current_block->initial_rate;
    this->trapezoid_tick_cycle_counter = 0;
    // Because this can be called directly from the main loop, it could be interrupted by the acceleration ticker, and that would be bad, so we use a flag
    this->trapezoid_generator_busy = true;
    this->set_step_events_per_minute(this->trapezoid_adjusted_rate); 
    this->trapezoid_generator_busy = false;
}

void Stepper::set_step_events_per_minute( double steps_per_minute ){

    // We do not step slower than this 
    steps_per_minute = max(steps_per_minute, this->minimum_steps_per_minute);

    // The speed factor is the factor by which we must multiply the minimal step frequency to reach the maximum step frequency
    // The higher, the smoother the movement will be, but the closer the maximum and minimum frequencies are, the smaller the factor is
    // speed_factor = base_stepping_frequency / steps_per_second
    // counter_increment = 1 / speed_factor ( 1 is 1<<16 because we do fixed point )
    this->counter_increment = int(floor(double(1<<16)/double(this->base_stepping_frequency / (steps_per_minute/60L))));

    this->kernel->call_event(ON_SPEED_CHANGE, this);

}

uint32_t Stepper::reset_step_pins(uint32_t dummy){
    this->alpha_step_pin->set(0);
    this->beta_step_pin->set(0); 
    this->gamma_step_pin->set(0);

}
