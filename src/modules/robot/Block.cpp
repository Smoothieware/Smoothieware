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
    clear_vector(this->speeds);
}

void Block::debug(Kernel* kernel){
    kernel->serial->printf(" steps:%4d|%4d|%4d(max:%4d) speeds:%8.2f|%8.2f|%8.2f nominal:r%10d/s%6.1f mm:%9.6f rdelta:%8d acc:%5d dec:%5d factor:%6.4f rates:%10d>%10d \r\n", this->steps[0], this->steps[1], this->steps[2], this->steps_event_count, this->speeds[0], this->speeds[1], this->speeds[2], this->nominal_rate, this->nominal_speed, this->millimeters, this->rate_delta, this->accelerate_until, this->decelerate_after, this->entry_factor, this->initial_rate, this->final_rate );
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
    this->initial_rate = ceil(this->nominal_rate * entryfactor); 
    this->final_rate   = ceil(this->nominal_rate * exitfactor);
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

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal
// velocities of the respective blocks.
inline double junction_jerk(Block* before, Block* after) {
    //printf("    compute jerk: before_speed_x: %f, after_speed_x: %f, difference: %f, pow: %f \r\n", before->speeds[X_AXIS], after->speeds[X_AXIS], before->speeds[X_AXIS]-after->speeds[X_AXIS], pow(before->speeds[X_AXIS]-after->speeds[X_AXIS], 2) );
    return(sqrt(
                pow(before->speeds[X_AXIS]-after->speeds[X_AXIS], 2)+
                pow(before->speeds[Y_AXIS]-after->speeds[Y_AXIS], 2)+
                pow(before->speeds[Z_AXIS]-after->speeds[Z_AXIS], 2))
          );
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

    double entryfactor = 1.0;
    double exitfactor;
    if (next) { exitfactor = next->entry_factor; } else { exitfactor = this->compute_factor_for_safe_speed(); }

    // Calculate the entry_factor for the this block.
    if (previous) {
        // Reduce speed so that junction_jerk is within the maximum allowed
        double jerk = junction_jerk(previous, this);
        if (jerk > this->planner->max_jerk ) {    //TODO: Get from config settings.max_jerk
            entryfactor = (this->planner->max_jerk/jerk);
        }
        // If the required deceleration across the block is too rapid, reduce the entry_factor accordingly.
        if (entryfactor > exitfactor) {
            double max_entry_speed = max_allowable_speed(-this->planner->acceleration,this->nominal_speed*exitfactor, this->millimeters); 
            double max_entry_factor = max_entry_speed/this->nominal_speed;
            if (max_entry_factor < entryfactor) {
                entryfactor = max_entry_factor;
            }
        }
    } else {
        entryfactor = this->compute_factor_for_safe_speed();
    }
    this->entry_factor = entryfactor;
}




// Called by Planner::recalculate() when scanning the plan from first to last entry.
void Block::forward_pass(Block* previous, Block* next){
  if(previous) {
      // If the previous block is an acceleration block, but it is not long enough to
      // complete the full speed change within the block, we need to adjust out entry
      // speed accordingly. Remember this->entry_factor equals the exit factor of
      // the previous block.
      if(previous->entry_factor < this->entry_factor) {
          double max_entry_speed = max_allowable_speed(-this->planner->acceleration, this->nominal_speed*previous->entry_factor, previous->millimeters); 
          double max_entry_factor = max_entry_speed/this->nominal_speed;
          if (max_entry_factor < this->entry_factor) {
              this->entry_factor = max_entry_factor;
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


