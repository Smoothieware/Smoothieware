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
#include <string>
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/StreamOutputPool.h"
#include "Stepper.h"
#include "StepTicker.h"

#include "mri.h"

using std::string;
#include <vector>

#define STEP_TICKER_FREQUENCY THEKERNEL->step_ticker->get_frequency()
#define STEP_TICKER_FREQUENCY_2 (STEP_TICKER_FREQUENCY*STEP_TICKER_FREQUENCY)

// A block represents a movement, it's length for each stepper motor, and the corresponding acceleration curves.
// It's stacked on a queue, and that queue is then executed in order, to move the motors.
// Most of the accel math is also done in this class
// And GCode objects for use in on_gcode_execute are also help in here

Block::Block()
{
    clear();
}

void Block::clear()
{
    //commands.clear();
    //travel_distances.clear();
    gcodes.clear();
    std::vector<Gcode>().swap(gcodes); // this resizes the vector releasing its memory

    this->steps.fill(0);

    steps_event_count   = 0;
    nominal_rate        = 0;
    nominal_speed       = 0.0F;
    millimeters         = 0.0F;
    entry_speed         = 0.0F;
    exit_speed          = 0.0F;
    acceleration        = 100.0F; // we don't want to get devide by zeroes if this is not set
    initial_rate        = -1;
    accelerate_until    = 0;
    decelerate_after    = 0;
    direction_bits      = 0;
    recalculate_flag    = false;
    nominal_length_flag = false;
    max_entry_speed     = 0.0F;
    is_ready            = false;
    times_taken         = 0;
    acceleration_per_tick= 0;
    deceleration_per_tick= 0;
    total_move_ticks= 0;
}

void Block::debug()
{
    THEKERNEL->streams->printf("%p: steps:X%04lu Y%04lu Z%04lu(max:%4lu) nominal:r%10lu/s%6.1f mm:%9.6f acc:%5lu dec:%5lu rates:%10lu  entry/max: %10.4f/%10.4f taken:%d ready:%d recalc:%d nomlen:%d\r\n",
                               this,
                               this->steps[0],
                               this->steps[1],
                               this->steps[2],
                               this->steps_event_count,
                               this->nominal_rate,
                               this->nominal_speed,
                               this->millimeters,
                               this->accelerate_until,
                               this->decelerate_after,
                               this->initial_rate,
                               this->entry_speed,
                               this->max_entry_speed,
                               this->times_taken,
                               this->is_ready,
                               recalculate_flag ? 1 : 0,
                               nominal_length_flag ? 1 : 0
                              );
}


/* Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
//                                +--------+ <- nominal_rate
//                               /          \
// nominal_rate*entry_factor -> +            \
//                              |             + <- nominal_rate*exit_factor
//                              +-------------+
//                                  time -->
*/
void Block::calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (times_taken)
        return;

    float initial_rate = this->nominal_rate * (entryspeed / this->nominal_speed); // steps/sec
    float final_rate = this->nominal_rate * (exitspeed / this->nominal_speed);
    //printf("Initial rate: %f, final_rate: %f\n", initial_rate, final_rate);
    // How many steps ( can be fractions of steps, we need very precise values ) to accelerate and decelerate
    // This is a simplification to get rid of rate_delta and get the steps/s² accel directly from the mm/s² accel
    float acceleration_per_second = (this->acceleration * this->steps_event_count) / this->millimeters;

    float maximum_possible_rate = sqrtf( ( this->steps_event_count * acceleration_per_second ) + ( ( powf(initial_rate, 2) + powf(final_rate, 2) ) / 2.0F ) );

    //printf("id %d: acceleration_per_second: %f, maximum_possible_rate: %f steps/sec, %f mm/sec\n", this->id, acceleration_per_second, maximum_possible_rate, maximum_possible_rate/100);

    // Now this is the maximum rate we'll achieve this move, either because
    // it's the higher we can achieve, or because it's the higher we are
    // allowed to achieve
    this->maximum_rate = std::min(maximum_possible_rate, (float)this->nominal_rate);

    // Now figure out how long it takes to accelerate in seconds
    float time_to_accelerate = ( this->maximum_rate - initial_rate ) / acceleration_per_second;

    // Now figure out how long it takes to decelerate
    float time_to_decelerate = ( final_rate -  this->maximum_rate ) / -acceleration_per_second;

    // Now we know how long it takes to accelerate and decelerate, but we must
    // also know how long the entire move takes so we can figure out how long
    // is the plateau if there is one
    float plateau_time = 0;

    // Only if there is actually a plateau ( we are limited by nominal_rate )
    if(maximum_possible_rate > this->nominal_rate) {
        // Figure out the acceleration and deceleration distances ( in steps )
        float acceleration_distance = ( ( initial_rate + this->maximum_rate ) / 2.0F ) * time_to_accelerate;
        float deceleration_distance = ( ( this->maximum_rate + final_rate ) / 2.0F ) * time_to_decelerate;

        // Figure out the plateau steps
        float plateau_distance = this->steps_event_count - acceleration_distance - deceleration_distance;

        // Figure out the plateau time in seconds
        plateau_time = plateau_distance / this->maximum_rate;
    }

    // Figure out how long the move takes total ( in seconds )
    float total_move_time = time_to_accelerate + time_to_decelerate + plateau_time;
    //puts "total move time: #{total_move_time}s time to accelerate: #{time_to_accelerate}, time to decelerate: #{time_to_decelerate}"

    // We now have the full timing for acceleration, plateau and deceleration,
    // yay \o/ Now this is very important these are in seconds, and we need to
    // round them into ticks. This means instead of accelerating in 100.23
    // ticks we'll accelerate in 100 ticks. Which means to reach the exact
    // speed we want to reach, we must figure out a new/slightly different
    // acceleration/deceleration to be sure we accelerate and decelerate at
    // the exact rate we want

    // First off round total time, acceleration time and deceleration time in ticks
    uint32_t acceleration_ticks = floorf( time_to_accelerate * STEP_TICKER_FREQUENCY );
    uint32_t deceleration_ticks = floorf( time_to_decelerate * STEP_TICKER_FREQUENCY );
    uint32_t total_move_ticks   = floorf( total_move_time    * STEP_TICKER_FREQUENCY );

    // Now deduce the plateau time for those new values expressed in tick
    //uint32_t plateau_ticks = total_move_ticks - acceleration_ticks - deceleration_ticks;

    // Now we figure out the acceleration value to reach EXACTLY maximum_rate(steps/s) in EXACTLY acceleration_ticks(ticks) amount of time in seconds
    float acceleration_time = acceleration_ticks / STEP_TICKER_FREQUENCY;  // This can be moved into the operation below, separated for clarity, note we need to do this instead of using time_to_accelerate(seconds) directly because time_to_accelerate(seconds) and acceleration_ticks(seconds) do not have the same value anymore due to the rounding
    float deceleration_time = deceleration_ticks / STEP_TICKER_FREQUENCY;

    float acceleration_in_steps = (acceleration_time > 0.0F ) ? ( this->maximum_rate - initial_rate ) / acceleration_time : 0;
    float deceleration_in_steps =  (deceleration_time > 0.0F ) ? ( this->maximum_rate - final_rate ) / deceleration_time : 0;

    // Note we use this value for acceleration as well as for deceleration, if that doesn't work, we can also as well compute the deceleration value this way :
    // float deceleration(steps/s²) = ( final_rate(steps/s) - maximum_rate(steps/s) ) / acceleration_time(s);
    // and store that in the block and use it for deceleration, which -will- yield better results, but may not be useful. If the moves do not end correctly, try computing this value, adding it to the block, and then using it for deceleration in the step generator

    // Now figure out the two acceleration ramp change events in ticks
    this->accelerate_until = acceleration_ticks;
    this->decelerate_after = total_move_ticks - deceleration_ticks;

    // Now figure out the acceleration PER TICK, this should ideally be held as a float, even a double if possible as it's very critical to the block timing
    // steps/tick^2

    this->acceleration_per_tick =  acceleration_in_steps / STEP_TICKER_FREQUENCY_2;
    this->deceleration_per_tick = deceleration_in_steps / STEP_TICKER_FREQUENCY_2;

    // We now have everything we need for this block to call a Steppermotor->move method !!!!
    // Theorically, if accel is done per tick, the speed curve should be perfect.

    // We need this to call move()
    this->total_move_ticks = total_move_ticks;

    //puts "accelerate_until: #{this->accelerate_until}, decelerate_after: #{this->decelerate_after}, acceleration_per_tick: #{this->acceleration_per_tick}, total_move_ticks: #{this->total_move_ticks}"

    this->initial_rate = initial_rate;
    this->exit_speed = exitspeed;
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
float Block::estimate_acceleration_distance(float initialrate, float targetrate, float acceleration)
{
    return( ((targetrate * targetrate) - (initialrate * initialrate)) / (2.0F * acceleration));
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
float Block::intersection_distance(float initialrate, float finalrate, float acceleration, float distance)
{
    return((2 * acceleration * distance - initialrate * initialrate + finalrate * finalrate) / (4 * acceleration));
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Block::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}


// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->entry_speed != this->max_entry_speed) {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->nominal_length_flag) && (this->max_entry_speed > exit_speed)) {
            float max_entry_speed = max_allowable_speed(-this->acceleration, exit_speed, this->millimeters);

            this->entry_speed = min(max_entry_speed, this->max_entry_speed);

            return this->entry_speed;
        } else
            this->entry_speed = this->max_entry_speed;
    }

    return this->entry_speed;
}


// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > nominal_speed)
        prev_max_exit_speed = nominal_speed;
    if (prev_max_exit_speed > max_entry_speed)
        prev_max_exit_speed = max_entry_speed;

    if (prev_max_exit_speed <= entry_speed) {
        // accel limited
        entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}

float Block::max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if (times_taken)
        return exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (nominal_length_flag)
        return nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-this->acceleration, this->entry_speed, this->millimeters);

    return min(max, nominal_speed);
}

// Gcodes are attached to their respective blocks so that on_gcode_execute can be called with it
void Block::append_gcode(Gcode* gcode)
{
    Gcode new_gcode = *gcode;
    new_gcode.strip_parameters(); // optimization to save memory we strip off the XYZIJK parameters from the saved command
    gcodes.push_back(new_gcode);
}

void Block::begin()
{
    recalculate_flag = false;

    if (!is_ready)
        __debugbreak();

    times_taken = -1;

    // execute all the gcodes related to this block
    for(unsigned int index = 0; index < gcodes.size(); index++)
        THEKERNEL->call_event(ON_GCODE_EXECUTE, &(gcodes[index]));


    THEKERNEL->call_event(ON_BLOCK_BEGIN, this);

    if (times_taken < 0)
        release();
}

// Signal the conveyor that this block is ready to be injected into the system
void Block::ready()
{
    this->is_ready = true;
}

// Mark the block as taken by one more module
void Block::take()
{
    if (times_taken < 0)
        times_taken = 0;
    times_taken++;
}

// Mark the block as no longer taken by one module, go to next block if this frees it
void Block::release()
{
    if (--this->times_taken <= 0) {
        times_taken = 0;
        if (is_ready) {
            is_ready = false;
            THEKERNEL->call_event(ON_BLOCK_END, this);

            // ensure conveyor gets called last
            THEKERNEL->conveyor->on_block_end(this);
        }
    }
}
