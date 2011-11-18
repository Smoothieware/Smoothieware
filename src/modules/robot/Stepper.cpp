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
#include "mbed.h"
#include <vector>
using namespace std;
#include "libs/nuts_bolts.h"

Timeout flipper;
Stepper* stepper;

Stepper::Stepper(){
    this->current_block = NULL;
    this->step_events_completed = 0; 
    this->divider = 0;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded(){
    stepper = this;
    this->register_for_event(ON_STEPPER_WAKE_UP);
    this->register_for_event(ON_PLAY);
    this->register_for_event(ON_PAUSE);
 
    // Get onfiguration
    this->on_config_reload(this); 

    // Acceleration timer
    this->acceleration_ticker.attach_us(this, &Stepper::trapezoid_generator_tick, 1000000/this->acceleration_ticks_per_second);

    // Initiate main_interrupt timer and step reset timer
    LPC_TIM0->MR0 = 10000;
    LPC_TIM0->MCR = 11; // for MR0 and MR1, with no reset at MR1
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER3_IRQn, 1); 
    LPC_TIM0->TCR = 1; 
    
    // Step and Dir pins as outputs
    this->step_gpio_port->FIODIR |= this->step_mask;
    this->dir_gpio_port->FIODIR  |= this->dir_mask;

}

void Stepper::on_config_reload(void* argument){
    LPC_GPIO_TypeDef *gpios[5] ={LPC_GPIO0,LPC_GPIO1,LPC_GPIO2,LPC_GPIO3,LPC_GPIO4};
    this->microseconds_per_step_pulse   =  this->kernel->config->value(microseconds_per_step_pulse_ckeckusm  )->by_default(5     )->as_number();
    this->acceleration_ticks_per_second =  this->kernel->config->value(acceleration_ticks_per_second_checksum)->by_default(100   )->as_number();
    this->minimum_steps_per_minute      =  this->kernel->config->value(minimum_steps_per_minute_checksum     )->by_default(1200  )->as_number();
    this->base_stepping_frequency       =  this->kernel->config->value(base_stepping_frequency_checksum      )->by_default(100000)->as_number();
    this->step_gpio_port      = gpios[(int)this->kernel->config->value(step_gpio_port_checksum               )->by_default(0     )->as_number()];
    this->dir_gpio_port       = gpios[(int)this->kernel->config->value(dir_gpio_port_checksum                )->by_default(0     )->as_number()];
    this->alpha_step_pin                =  this->kernel->config->value(alpha_step_pin_checksum               )->required(        )->as_number();
    this->beta_step_pin                 =  this->kernel->config->value(beta_step_pin_checksum                )->required(        )->as_number();
    this->gamma_step_pin                =  this->kernel->config->value(gamma_step_pin_checksum               )->required(        )->as_number();
    this->alpha_dir_pin                 =  this->kernel->config->value(alpha_dir_pin_checksum                )->required(        )->as_number();
    this->beta_dir_pin                  =  this->kernel->config->value(beta_dir_pin_checksum                 )->required(        )->as_number();
    this->gamma_dir_pin                 =  this->kernel->config->value(gamma_dir_pin_checksum                )->required(        )->as_number();
    this->step_mask                     = ( 1 << this->alpha_step_pin ) + ( 1 << this->beta_step_pin ) + ( 1 << this->gamma_step_pin );
    this->dir_mask                      = ( 1 << this->alpha_dir_pin  ) + ( 1 << this->beta_dir_pin  ) + ( 1 << this->gamma_dir_pin  );
    this->step_bits[ALPHA_STEPPER ]     = this->alpha_step_pin;
    this->step_bits[BETA_STEPPER  ]     = this->beta_step_pin;
    this->step_bits[GAMMA_STEPPER ]     = this->gamma_step_pin;
    this->step_invert_mask = ( this->kernel->config->value( alpha_step_pin_checksum )->is_inverted() << this->alpha_step_pin ) +
                             ( this->kernel->config->value( beta_step_pin_checksum  )->is_inverted() << this->beta_step_pin  ) + 
                             ( this->kernel->config->value( gamma_step_pin_checksum )->is_inverted() << this->gamma_step_pin );
    this->dir_invert_mask =  ( this->kernel->config->value( alpha_dir_pin_checksum  )->is_inverted() << this->alpha_dir_pin  ) +
                             ( this->kernel->config->value( beta_dir_pin_checksum   )->is_inverted() << this->beta_dir_pin   ) + 
                             ( this->kernel->config->value( gamma_dir_pin_checksum  )->is_inverted() << this->gamma_dir_pin  );

    // Set the Timer interval for Match Register 1, 
    LPC_TIM0->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse; 
}

// When the play/pause button is set to pause, or a module calls the ON_PAUSE event
void Stepper::on_pause(void* argument){
    LPC_TIM0->TCR = 0;
    this->acceleration_ticker.detach();
}

// When the play/pause button is set to play, or a module calls the ON_PLAY event
void Stepper::on_play(void* argument){
    LPC_TIM0->TCR = 1;
    this->acceleration_ticker.attach_us(this, &Stepper::trapezoid_generator_tick, 1000000/this->acceleration_ticks_per_second);
}


// Timer0 ISR
// MR0 is used to call the main stepping interrupt, and MR1 to reset the stepping pins
extern "C" void TIMER0_IRQHandler (void){
    if((LPC_TIM0->IR >> 1) & 1){
        LPC_TIM0->IR |= 1 << 1;
        // Clear step pins 
        stepper->step_gpio_port->FIOSET =  stepper->step_invert_mask & stepper->step_mask;
        stepper->step_gpio_port->FIOCLR = ~stepper->step_invert_mask & stepper->step_mask; 
    }
    if((LPC_TIM0->IR >> 0) & 1){
        LPC_TIM0->IR |= 1 << 0;
        stepper->main_interrupt();
    }
}


//Called ( apriori only by the planner ) when Stepper asked to wake up : means we have work to do. Argument is the queue so that we can use it. //TODO : Get rid of this
void Stepper::on_stepper_wake_up(void* argument){
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Smoothie. It is executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
void Stepper::main_interrupt(){
    
    // Step dir pins first, then step pinse, stepper drivers like to know the direction before the step signal comes in
    // Clear dir   pins 
    this->dir_gpio_port->FIOSET  =                          this->dir_invert_mask          & this->dir_mask; 
    this->dir_gpio_port->FIOCLR  =     ~                    this->dir_invert_mask          & this->dir_mask;
    // Set   dir   pins
    this->dir_gpio_port->FIOSET  =   (     this->out_bits ^ this->dir_invert_mask    )     & this->dir_mask;
    this->dir_gpio_port->FIOCLR  =   ( ~ ( this->out_bits ^ this->dir_invert_mask  ) )     & this->dir_mask; 
    // Set   step  pins 
    this->step_gpio_port->FIOSET =   (     this->out_bits ^ this->step_invert_mask   )     & this->step_mask;  
    this->step_gpio_port->FIOCLR =   ( ~ ( this->out_bits ^ this->step_invert_mask ) )     & this->step_mask;

    // If there is no current block, attempt to pop one from the buffer, if there is none, go to sleep, if there is none, go to sleep
    if( this->current_block == NULL ){
        this->current_block = this->kernel->planner->get_current_block();
        if( this->current_block != NULL ){
            if( this->current_block->computed == false ){
               this->current_block = NULL;
            }else{ 
                this->trapezoid_generator_reset();
                this->update_offsets();
                for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ this->counters[stpr] = 0; this->stepped[stpr] = 0; } 
                this->step_events_completed = 0; 
                this->kernel->call_event(ON_BLOCK_BEGIN, this->current_block);
                //this->kernel->planner->dump_queue();
            } 
        }else{
            // Go to sleep
            //LPC_TIM0->MR0 = 10000;  
            //LPC_TIM0->MR1 = 500;
            //LPC_TIM0->TCR = 0;
        }
    }
   
    if( this->current_block != NULL ){
        // Set bits for direction and steps 
        this->out_bits = this->current_block->direction_bits;
        for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ 
            this->counters[stpr] += this->counter_increment; // (1<<16)>>this->divider; // Basically += 1/divider if we were in floating point arythmetic, but here the counters precision is 1/(2^16). 
            if( this->counters[stpr] > this->offsets[stpr] && this->stepped[stpr] < this->current_block->steps[stpr] ){
                this->counters[stpr] -= this->offsets[stpr] ;
                this->stepped[stpr]++;
                this->out_bits |= (1 << this->step_bits[stpr]);
            } 
        } 
        // If current block is finished, reset pointer
        this->step_events_completed += this->counter_increment; // (1<<16)>>this->divider; // /this->divider;
        if( this->step_events_completed >= this->current_block->steps_event_count<<16 ){ 
            if( this->stepped[ALPHA_STEPPER] == this->current_block->steps[ALPHA_STEPPER] && this->stepped[BETA_STEPPER] == this->current_block->steps[BETA_STEPPER] && this->stepped[GAMMA_STEPPER] == this->current_block->steps[GAMMA_STEPPER] ){ 
                this->kernel->call_event(ON_BLOCK_END, this->current_block);
                this->current_block->pop_and_execute_gcode(this->kernel);
                this->current_block = NULL;
                this->kernel->planner->discard_current_block();
            }
        }

    }else{
        this->out_bits = 0;
    }
}

void Stepper::update_offsets(){
    for( int stpr=ALPHA_STEPPER; stpr<=GAMMA_STEPPER; stpr++){ 
        this->offsets[stpr] = (int)floor((float)((float)(1<<16)*(float)((float)this->current_block->steps_event_count / (float)this->current_block->steps[stpr]))); 
    }
}


// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
void Stepper::trapezoid_generator_tick() {
    if(this->current_block && !this->kernel->planner->computing ) {
      if(this->step_events_completed < this->current_block->accelerate_until<<16) {
          this->trapezoid_adjusted_rate += this->current_block->rate_delta;
          if (this->trapezoid_adjusted_rate > this->current_block->nominal_rate ) {
              this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
          }
          this->set_step_events_per_minute(this->trapezoid_adjusted_rate);
      } else if (this->step_events_completed > this->current_block->decelerate_after<<16) {
          // NOTE: We will only reduce speed if the result will be > 0. This catches small
          // rounding errors that might leave steps hanging after the last trapezoid tick.
          if (this->trapezoid_adjusted_rate > this->current_block->rate_delta) {
              this->trapezoid_adjusted_rate -= this->current_block->rate_delta;
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
    this->set_step_events_per_minute(this->trapezoid_adjusted_rate); 
    this->trapezoid_generator_busy = false;
}

void Stepper::set_step_events_per_minute( double steps_per_minute ){

    // We do not step slower than this 
    if( steps_per_minute < this->minimum_steps_per_minute ){ steps_per_minute = this->minimum_steps_per_minute; }

    // The speed factor is the factor by which we must multiply the minimal step frequency to reach the maximum step frequency
    // The higher, the smoother the movement will be, but the closer the maximum and minimum frequencies are, the smaller the factor is
    double speed_factor = this->base_stepping_frequency / (steps_per_minute/60L);
    if( speed_factor < 1 ){ speed_factor=1; }
    this->counter_increment = int(floor((1<<16)/speed_factor));

    // TODO : Test if we don't get a better signal with a floored speed_factor

    // Set the Timer interval 
    LPC_TIM0->MR0 = floor( ( SystemCoreClock/4 ) / ( (steps_per_minute/60L) * speed_factor ) );
    
    // In case we change the Match Register to a value the Timer Counter has past
    if( LPC_TIM0->TC >= LPC_TIM0->MR0 ){ LPC_TIM0->TCR = 3; LPC_TIM0->TCR = 1; }

    this->kernel->call_event(ON_SPEED_CHANGE, this);
 
}

