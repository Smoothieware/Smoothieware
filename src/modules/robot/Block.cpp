/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <math.h>
#include "mbed.h"
#include <string>
#include "Block.h"
#include "Planner.h"
using std::string;
#include <vector>
#include "../communication/utils/Gcode.h"


Block::Block(){
    clear_vector(this->steps);
}

void Block::debug(Kernel* kernel){
    kernel->serial->printf(" steps:%4d|%4d|%4d(max:%4d) nominal:r%10d/s%6.1f mm:%9.6f rdelta:%8d acc:%5d dec:%5d rates:%10d>%10d \r\n", this->steps[0], this->steps[1], this->steps[2], this->steps_event_count, this->nominal_rate, this->nominal_speed, this->millimeters, this->rate_delta, this->accelerate_until, this->decelerate_after, this->initial_rate, this->final_rate );
}


// Calculate a braking factor to reach baseline speed which is max_jerk/2, e.g. the
// speed under which you cannot exceed max_jerk no matter what you do.
double Block::compute_factor_for_safe_speed(){
    return( this->planner->max_jerk / this->nominal_speed ); 
}


// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
//                                +--------+ <- nominal_rate
//                               /          \
// nominal_rate*entry_factor -> +            \
//                              |             + <- nominal_rate*exit_factor
//                              +-------------+
//                                  time -->
void Block::calculate_trapezoid( double entryfactor, double exitfactor ){
    this->initial_rate = ceil(this->nominal_rate * entryfactor);   // (step/min) 
    this->final_rate   = ceil(this->nominal_rate * exitfactor);    // (step/min)
    double acceleration_per_minute = this->rate_delta * this->planner->kernel->stepper->acceleration_ticks_per_second * 60.0; 
    int accelerate_steps = ceil( this->estimate_acceleration_distance( this->initial_rate, this->nominal_rate, acceleration_per_minute ) );
    int decelerate_steps = ceil( this->estimate_acceleration_distance( this->nominal_rate, this->final_rate,  -acceleration_per_minute ) );

    // Calculate the size of Plateau of Nominal Rate.
    int plateau_steps = this->steps_event_count-accelerate_steps-decelerate_steps;

   // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
   // have to use intersection_distance() to calculate when to abort acceleration and start braking
   // in order to reach the final_rate exactly at the end of this block.
   if (plateau_steps < 0) {
       accelerate_steps = ceil(this->intersection_distance(this->initial_rate, this->final_rate, acceleration_per_minute, this->steps_event_count));
       accelerate_steps = max( accelerate_steps, 0 ); // Check limits due to numerical round-off
       accelerate_steps = min( accelerate_steps, int(this->steps_event_count) );
       plateau_steps = 0;
   }
   
   this->accelerate_until = accelerate_steps;
   this->decelerate_after = accelerate_steps+plateau_steps; 

}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
double Block::estimate_acceleration_distance(double initialrate, double targetrate, double acceleration) {
      return( (targetrate*targetrate-initialrate*initialrate)/(2L*acceleration));
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
//
/*                          + <- some maximum rate we don't care about
                           /|\
                          / | \
                         /  |  + <- final_rate
                        /   |  |
       initial_rate -> +----+--+
                            ^ ^
                            | |
        intersection_distance distance */
double Block::intersection_distance(double initialrate, double finalrate, double acceleration, double distance) {
   return((2*acceleration*distance-initialrate*initialrate+finalrate*finalrate)/(4*acceleration));
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
inline double max_allowable_speed(double acceleration, double target_velocity, double distance) {
  return(
    sqrt(target_velocity*target_velocity-2L*acceleration*60*60*distance)  //Was acceleration*60*60*distance, in case this breaks, but here we prefer to use seconds instead of minutes
  );
}


// Called by Planner::recalculate() when scanning the plan from last to first entry.
void Block::reverse_pass(Block* next, Block* previous){

    if (next) {
        // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
        // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
        // check for maximum allowable speed reductions to ensure maximum possible planned speed.
        if (this->entry_speed != this->max_entry_speed) {

            // If nominal length true, max junction speed is guaranteed to be reached. Only compute
            // for max allowable speed if block is decelerating and nominal length is false.
            if ((!this->nominal_length_flag) && (this->max_entry_speed > next->entry_speed)) {
                this->entry_speed = min( this->max_entry_speed, max_allowable_speed(-this->planner->acceleration,next->entry_speed,this->millimeters));
            } else {
                this->entry_speed = this->max_entry_speed;
            }
            this->recalculate_flag = true;

        }
    } // Skip last block. Already initialized and set for recalculation.

}


// Called by Planner::recalculate() when scanning the plan from first to last entry.
void Block::forward_pass(Block* previous, Block* next){

    if(!previous) { return; } // Begin planning after buffer_tail

    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag) {
        if (previous->entry_speed < this->entry_speed) {
          double entry_speed = min( this->entry_speed,
            max_allowable_speed(-this->planner->acceleration,previous->entry_speed,previous->millimeters) );

          // Check for junction speed change
          if (this->entry_speed != entry_speed) {
            this->entry_speed = entry_speed;
            this->recalculate_flag = true;
          }
        }
    }
      
}


// Gcodes are attached to their respective blocks so that on_gcode_execute can be called with it
void Block::append_gcode(Gcode* gcode){
   this->commands.insert(this->commands.begin(),gcode->command);
}

// The attached gcodes are then poped and the on_gcode_execute event is called with them as a parameter
void Block::pop_and_execute_gcode(Kernel* &kernel){
    while( this->commands.size() > 0 ){
        string command = this->commands.back();
        Gcode gcode = Gcode();
        gcode.command = command;
        kernel->call_event(ON_GCODE_EXECUTE, &gcode ); 
        this->commands.pop_back();
    }
}


