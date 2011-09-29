/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

using namespace std;
#include <vector>
#include "mbed.h"
#include "libs/nuts_bolts.h"
#include "libs/RingBuffer.h"
#include "../communication/utils/Gcode.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Block.h"
#include "Planner.h"

Planner::Planner(){
    clear_vector(this->position); 
    this->has_deleted_block = false;
}

void Planner::on_module_loaded(){
    this->on_config_reload(this);
}

void Planner::on_config_reload(void* argument){
    this->acceleration = this->kernel->config->get(acceleration_checksum);
    this->max_jerk     = this->kernel->config->get(max_jerk_checksum    );
}


// Append a block to the queue, compute it's speed factors
void Planner::append_block( int target[], double feed_rate, double distance, double speeds[] ){
    
    // TODO : Check if this is necessary
    this->computing = true;
    
    // Do not append block with no movement
    if( target[ALPHA_STEPPER] == this->position[ALPHA_STEPPER] && target[BETA_STEPPER] == this->position[BETA_STEPPER] && target[GAMMA_STEPPER] == this->position[GAMMA_STEPPER] ){ this->computing = false; return; }
    
    // Stall here if the queue is full
    while( this->queue.size() >= this->queue.capacity() ){ wait_us(100); }     
    
    // Add/get a new block from the queue
    this->queue.push_back(Block());
    Block* block = this->queue.get_ref( this->queue.size()-1 );
    block->planner = this;
    
    // Number of steps for each stepper
    for( int stepper=ALPHA_STEPPER; stepper<=GAMMA_STEPPER; stepper++){ block->steps[stepper] = labs(target[stepper] - this->position[stepper]); } 
    
    // Max number of steps, for all axes
    block->steps_event_count = max( block->steps[ALPHA_STEPPER], max( block->steps[BETA_STEPPER], block->steps[GAMMA_STEPPER] ) );

    // Speeds for each axis, needed for junction_jerk
    for( int stepper=ALPHA_STEPPER; stepper<=GAMMA_STEPPER; stepper++){ block->speeds[stepper] = speeds[stepper]; } 

    // Set rates and speeds
    double microseconds = ( distance / feed_rate ) * 1000000;
    double multiplier   = 60*1000000/microseconds;
    block->nominal_rate = block->steps_event_count*multiplier;
    block->nominal_speed = distance*multiplier;
    block->millimeters = distance;
    block->entry_factor = 0;
    
    // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
    // average travel per step event changes. For a line along one axis the travel per step event
    // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
    // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
    // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed
    // specifically for each line to compensate for this phenomenon:
    double travel_per_step = (double) ((double) block->millimeters / (double) block->steps_event_count);
    block->rate_delta = ceil(( (this->acceleration*60) / this->kernel->stepper->acceleration_ticks_per_second ) / travel_per_step);

    // Comupte a preliminary conservative acceleration trapezoid
    double safe_speed_factor = block->compute_factor_for_safe_speed();
    block->calculate_trapezoid( safe_speed_factor, safe_speed_factor );
    
    // Direction bits
    block->direction_bits = 0; 
    char direction_bits[3] = {this->kernel->stepper->alpha_dir_pin, this->kernel->stepper->beta_dir_pin, this->kernel->stepper->gamma_dir_pin}; 
    for( int stepper=ALPHA_STEPPER; stepper<=GAMMA_STEPPER; stepper++){ 
        if( target[stepper] < position[stepper] ){ block->direction_bits |= (1<<direction_bits[stepper]); } 
    }
    
    memcpy(this->position, target, sizeof(int)*3);
    this->recalculate();
    this->computing = false;
    this->kernel->call_event(ON_STEPPER_WAKE_UP, this);
}

// Gcodes are attached to their respective block in the queue so that the on_gcode_execute event can be called with the gcode when the block is executed
void Planner::attach_gcode_to_queue(Gcode* gcode){
    //If the queue is empty, execute immediatly, otherwise attach to the last added block
    if( this->queue.size() == 0 ){
        this->kernel->call_event(ON_GCODE_EXECUTE, gcode ); 
    }else{
        this->queue.get_ref( this->queue.size() - 1 )->append_gcode(gcode);
    } 
}


// Recalculates the motion plan according to the following algorithm:
//
// 1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
// so that:
//   a. The junction jerk is within the set limit
//   b. No speed reduction within one block requires faster deceleration than the one, true constant
//      acceleration.
// 2. Go over every block in chronological order and dial down junction speed reduction values if
//   a. The speed increase within one block would require faster accelleration than the one, true
//      constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
// the set limit. Finally it will:
//
// 3. Recalculate trapezoids for all blocks.
//
void Planner::recalculate() {
   this->reverse_pass();
   this->forward_pass();
   this->recalculate_trapezoids();
}

// Planner::recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the reverse pass.
void Planner::reverse_pass(){
     // For each block
    for( int index = this->queue.size()-1; index >= 0; index-- ){
        //this->queue.get_ref(index)->reverse_pass((index==this->queue.size()-1?NULL:this->queue.get_ref(index+1)), (index==0?NULL:this->queue.get_ref(index-1))); 
        this->queue.get_ref(index)->reverse_pass((index==this->queue.size()-1?NULL:this->queue.get_ref(index+1)), (index==0? (this->has_deleted_block?&(this->last_deleted_block):NULL) :this->queue.get_ref(index-1))); 
    }
}

// Planner::recalculate() needs to go over the current plan twice. Once in reverse and once forward. This
// implements the forward pass.
void Planner::forward_pass() {
     // For each block
    for( int index = 0; index <= this->queue.size()-1; index++ ){
        this->queue.get_ref(index)->forward_pass((index==0?NULL:this->queue.get_ref(index-1)),(index==this->queue.size()-1?NULL:this->queue.get_ref(index+1))); 
    }
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the
// entry_factor for each junction. Must be called by Planner::recalculate() after
// updating the blocks.
void Planner::recalculate_trapezoids() {
    // For each block
    for( int index = 0; index <= this->queue.size()-1; index++ ){

        if( this->queue.size()-1 == index ){ //last block
           this->queue.get_ref(index)->calculate_trapezoid( this->queue.get_ref(index)->entry_factor, this->queue.get_ref(index)->compute_factor_for_safe_speed() ); 
        }else{
           this->queue.get_ref(index)->calculate_trapezoid( this->queue.get_ref(index)->entry_factor, this->queue.get_ref(index+1)->entry_factor ); 
        } 
    }
}

// Get the first block in the queue, or return NULL
Block* Planner::get_current_block(){
    if( this->queue.size() == 0 ){ return NULL; }
    return this->queue.get_ref(0);
}


// We are done with this block, discard it
void Planner::discard_current_block(){
    this->has_deleted_block = true;
    this->queue.get(0,this->last_deleted_block );
    this->queue.delete_first();
}

// Debug function
void Planner::dump_queue(){
    for( int index = 0; index <= this->queue.size()-1; index++ ){
       if( index > 10 && index < this->queue.size()-10 ){ continue; }
       this->kernel->serial->printf("block %03d > ", index);
       this->queue.get_ref(index)->debug(this->kernel); 
    }
}
