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
    clear_vector_double(this->previous_unit_vec);
    this->previous_nominal_speed = 0.0;
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
void Planner::append_block( int target[], double feed_rate, double distance, double deltas[] ){
   
    // TODO : Check if this is necessary
    
    // Do not append block with no movement
    if( target[ALPHA_STEPPER] == this->position[ALPHA_STEPPER] && target[BETA_STEPPER] == this->position[BETA_STEPPER] && target[GAMMA_STEPPER] == this->position[GAMMA_STEPPER] ){ this->computing = false; return; }
   
    // Stall here if the queue is ful
    while( this->queue.size() >= this->queue.capacity() ){ wait_us(100); }     
    
    // Add/get a new block from the queue
    this->queue.push_back(Block());
    Block* block = this->queue.get_ref( this->queue.size()-1 );
    block->planner = this;

    this->computing = true;

    // Direction bits
    block->direction_bits = 0; 
    char direction_bits[3] = {this->kernel->stepper->alpha_dir_pin, this->kernel->stepper->beta_dir_pin, this->kernel->stepper->gamma_dir_pin}; 
    for( int stepper=ALPHA_STEPPER; stepper<=GAMMA_STEPPER; stepper++){ 
        if( target[stepper] < position[stepper] ){ block->direction_bits |= (1<<direction_bits[stepper]); } 
    }
    
    // Number of steps for each stepper
    for( int stepper=ALPHA_STEPPER; stepper<=GAMMA_STEPPER; stepper++){ block->steps[stepper] = labs(target[stepper] - this->position[stepper]); } 
    
    // Max number of steps, for all axes
    block->steps_event_count = max( block->steps[ALPHA_STEPPER], max( block->steps[BETA_STEPPER], block->steps[GAMMA_STEPPER] ) );
    if( block->steps_event_count == 0 ){ this->computing = false; return; }

    block->millimeters = distance;
    double inverse_millimeters = 1.0/distance;

    // Calculate speed in mm/minute for each axis. No divide by zero due to previous checks.
    // NOTE: Minimum stepper speed is limited by MINIMUM_STEPS_PER_MINUTE in stepper.c
    double inverse_minute = feed_rate * inverse_millimeters;
    block->nominal_speed = block->millimeters * inverse_minute;           // (mm/min) Always > 0
    block->nominal_rate = ceil(block->steps_event_count * inverse_minute); // (step/min) Always > 0

    // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
    // average travel per step event changes. For a line along one axis the travel per step event
    // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
    // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
    // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed
    // specifically for each line to compensate for this phenomenon:
    // Convert universal acceleration for direction-dependent stepper rate change parameter
    block->rate_delta = ceil( block->steps_event_count*inverse_millimeters * this->acceleration*60.0 / this->kernel->stepper->acceleration_ticks_per_second ); // (step/min/acceleration_tick)


    // Compute path unit vector
    double unit_vec[3];
    unit_vec[X_AXIS] = deltas[X_AXIS]*inverse_millimeters;
    unit_vec[Y_AXIS] = deltas[Y_AXIS]*inverse_millimeters;
    unit_vec[Z_AXIS] = deltas[Z_AXIS]*inverse_millimeters;
  
    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

    if (this->queue.size() > 1 && (this->previous_nominal_speed > 0.0)) {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      double cos_theta = - this->previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                         - this->previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                         - this->previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                           
      // Skip and use default max junction speed for 0 degree acute junction.
      if (cos_theta < 0.95) {
        vmax_junction = min(this->previous_nominal_speed,block->nominal_speed);
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        if (cos_theta > -0.95) {
          // Compute maximum junction velocity based on maximum acceleration and junction deviation
          double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
          vmax_junction = min(vmax_junction,
            sqrt(this->acceleration*60*60 * 0.05 * sin_theta_d2/(1.0-sin_theta_d2)) ); // TODO: Get from config
        }
      }
    }
    block->max_entry_speed = vmax_junction;
   
    // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
    double v_allowable = this->max_allowable_speed(-this->acceleration,0.0,block->millimeters); //TODO: Get from config
    block->entry_speed = min(vmax_junction, v_allowable);

    // Initialize planner efficiency flags
    // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
    // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
    // the current block and next block junction speeds are guaranteed to always be at their maximum
    // junction speeds in deceleration and acceleration, respectively. This is due to how the current
    // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
    // the reverse and forward planners, the corresponding block junction speed will always be at the
    // the maximum junction speed and may always be ignored for any speed reduction checks.
    if (block->nominal_speed <= v_allowable) { block->nominal_length_flag = true; }
    else { block->nominal_length_flag = false; }
    block->recalculate_flag = true; // Always calculate trapezoid for new block
 

    // Update previous path unit_vector and nominal speed
    memcpy(this->previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
    this->previous_nominal_speed = block->nominal_speed;

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
    for( int index = this->queue.size()-1; index > 0; index-- ){  // Skip buffer tail/first block to prevent over-writing the initial entry speed.
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

// Recalculates the trapezoid speed profiles for flagged blocks in the plan according to the
// entry_speed for each junction and the entry_speed of the next junction. Must be called by
// planner_recalculate() after updating the blocks. Any recalulate flagged junction will
// compute the two adjacent trapezoids to the junction, since the junction speed corresponds
// to exit speed and entry speed of one another.
void Planner::recalculate_trapezoids() {
    // For each block
    for( int index = 0; index <= this->queue.size()-1; index++ ){ // We skip the first one because we need a previous

        if( this->queue.size()-1 == index ){ //last block
            Block* last = this->queue.get_ref(index);
            last->calculate_trapezoid( last->entry_speed / last->nominal_speed, MINIMUM_PLANNER_SPEED / last->nominal_speed ); 
        }else{
            Block* current = this->queue.get_ref(index);
            Block* next =    this->queue.get_ref(index+1);
            if( current->recalculate_flag || next->recalculate_flag ){
                current->calculate_trapezoid( current->entry_speed/current->nominal_speed, next->entry_speed/current->nominal_speed );
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            } 
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



// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
double Planner::max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return(
    sqrt(target_velocity*target_velocity-2L*acceleration*60*60*distance)  //Was acceleration*60*60*distance, in case this breaks, but here we prefer to use seconds instead of minutes
  );
}


