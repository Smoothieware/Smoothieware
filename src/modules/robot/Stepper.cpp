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
#include "Block.h"
#include "StepTicker.h"

#include <vector>
using namespace std;

#include "libs/nuts_bolts.h"
#include "libs/Hook.h"

#include <mri.h>

// The stepper reacts to blocks that have XYZ movement to transform them into actual stepper motor moves
// TODO: This does accel, accel should be in StepperMotor

Stepper::Stepper()
{
    this->current_block = NULL;
    this->force_speed_update = false;
    this->halted= false;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded()
{
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    // Get onfiguration
    this->on_config_reload(this);

    // Acceleration ticker
    THEKERNEL->step_ticker->register_acceleration_tick_handler([this](){trapezoid_generator_tick(); });

    // Attach to the end_of_move stepper event
    for (auto actuator : THEKERNEL->robot->actuators)
        actuator->attach(this, &Stepper::stepper_motor_finished_move );
}

// Get configuration from the config file
void Stepper::on_config_reload(void *argument)
{
    // Steppers start off by default
    this->turn_enable_pins_off();
}

void Stepper::on_halt(void *argument)
{
    if(argument == nullptr) {
        this->turn_enable_pins_off();
        this->halted= true;
    }else{
        this->halted= false;
    }
}

void Stepper::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    // Attach gcodes to the last block for on_gcode_execute
    if( gcode->has_m && (gcode->m == 84 || gcode->m == 17 || gcode->m == 18 )) {
        THEKERNEL->conveyor->append_gcode(gcode);
    }
}

// React to enable/disable gcodes
void Stepper::on_gcode_execute(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        if( gcode->m == 17 ) {
            this->turn_enable_pins_on();
        }
        if( (gcode->m == 84 || gcode->m == 18) && !gcode->has_letter('E') ) {
            this->turn_enable_pins_off();
        }
    }
}

// Enable steppers
void Stepper::turn_enable_pins_on()
{
    for (auto a : THEKERNEL->robot->actuators)
        a->enable(true);
    this->enable_pins_status = true;
    THEKERNEL->call_event(ON_ENABLE, (void*)1);
}

// Disable steppers
void Stepper::turn_enable_pins_off()
{
    for (auto a : THEKERNEL->robot->actuators)
        a->enable(false);
    this->enable_pins_status = false;
    THEKERNEL->call_event(ON_ENABLE, nullptr);
}

// A new block is popped from the queue
void Stepper::on_block_begin(void *argument)
{
    Block *block  = static_cast<Block *>(argument);

    // Mark the new block as of interrest to us, handle blocks that have no axis moves properly (like Extrude blocks etc)
    bool take = false;
    if (block->millimeters > 0.0F) {
        for (size_t s = 0; !take && s < THEKERNEL->robot->actuators.size(); s++) {
            take = block->steps[s] > 0;
        }
    }
    if (take){
        block->take();
    } else {
        // none of the steppers move this block so make sure they know that
        for(auto a : THEKERNEL->robot->actuators) {
            a->set_moved_last_block(false);
        }
        return;
    }

    // We can't move with the enable pins off
    if( this->enable_pins_status == false ) {
        this->turn_enable_pins_on();
    }

    // Setup : instruct stepper motors to move
    // Find the stepper with the more steps, it's the one the speed calculations will want to follow
    this->main_stepper = nullptr;
    int most_steps_to_move = 0;
    for (size_t i = 0; i < THEKERNEL->robot->actuators.size(); i++) {
        if (block->steps[i] > 0) {
            THEKERNEL->robot->actuators[i]->move(block->direction_bits[i], block->steps[i])->set_moved_last_block(true);
            int steps_to_move = THEKERNEL->robot->actuators[i]->get_steps_to_move();
            if (steps_to_move > most_steps_to_move) {
                most_steps_to_move = steps_to_move;
                this->main_stepper = THEKERNEL->robot->actuators[i];
            }
        }
        else {
            THEKERNEL->robot->actuators[i]->set_moved_last_block(false);
        }
    }

    this->current_block = block;

    // Setup acceleration for this block
    this->trapezoid_generator_reset();

    // Set the initial speed for this move
    this->trapezoid_generator_tick();

    // synchronize the acceleration timer with the start of the new block so it does not drift and randomly fire during the block
    THEKERNEL->step_ticker->synchronize_acceleration(false);

    // set a flag to synchronize the acceleration timer with the deceleration step, and fire it immediately we get to that step
    if( block->decelerate_after > 0 && block->decelerate_after+1 < this->main_stepper->steps_to_move ) {
        this->main_stepper->signal_step= block->decelerate_after+1; // we make it +1 as deceleration does not start until steps > decelerate_after
    }
}

// Current block is discarded
void Stepper::on_block_end(void *argument)
{
    this->current_block = NULL; //stfu !
}

// When a stepper motor has finished it's assigned movement
uint32_t Stepper::stepper_motor_finished_move(uint32_t dummy)
{
    // We care only if none is still moving
    for (auto a : THEKERNEL->robot->actuators) {
        if(a->moving)    
            return 0;
    }

    // This block is finished, release it
    if( this->current_block != NULL ) {
        this->current_block->release();
    }

    return 0;
}


// This is called ACCELERATION_TICKS_PER_SECOND times per second by the step_event
// interrupt. It can be assumed that the trapezoid-generator-parameters and the
// current_block stays untouched by outside handlers for the duration of this function call.
// NOTE caled at the same priority as PendSV so it may make that longer but it is better that having htis pre empted by pendsv
void Stepper::trapezoid_generator_tick(void)
{
    // Do not do the accel math for nothing
    if(this->current_block && this->main_stepper->moving ) {

        // Store this here because we use it a lot down there
        uint32_t current_steps_completed = this->main_stepper->stepped;
        float last_rate= trapezoid_adjusted_rate;

        if( this->force_speed_update ) {
            // Do not accel, just set the value
            this->force_speed_update = false;
            last_rate= -1;

        } else if(THEKERNEL->conveyor->is_flushing()) {
            // if we are flushing the queue, decelerate to 0 then finish this block
            if (trapezoid_adjusted_rate > current_block->rate_delta * 1.5F) {
                trapezoid_adjusted_rate -= current_block->rate_delta;

            } else if (trapezoid_adjusted_rate == current_block->rate_delta * 0.5F) {
                for (auto i : THEKERNEL->robot->actuators) i->move(i->direction, 0); // stop motors
                if (current_block) current_block->release();
                THEKERNEL->call_event(ON_SPEED_CHANGE, 0); // tell others we stopped
                return;

            } else {
                trapezoid_adjusted_rate = current_block->rate_delta * 0.5F;
            }

        } else if(current_steps_completed <= this->current_block->accelerate_until) {
            // If we are accelerating
            // Increase speed
            this->trapezoid_adjusted_rate += this->current_block->rate_delta;
            if (this->trapezoid_adjusted_rate > this->current_block->nominal_rate ) {
                this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
            }

        } else if (current_steps_completed > this->current_block->decelerate_after) {
            // If we are decelerating
            // Reduce speed
            // NOTE: We will only reduce speed if the result will be > 0. This catches small
            // rounding errors that might leave steps hanging after the last trapezoid tick.
            if(this->trapezoid_adjusted_rate > this->current_block->rate_delta * 1.5F) {
                this->trapezoid_adjusted_rate -= this->current_block->rate_delta;
            } else {
                this->trapezoid_adjusted_rate = this->current_block->rate_delta * 1.5F;
            }
            if(this->trapezoid_adjusted_rate < this->current_block->final_rate ) {
                this->trapezoid_adjusted_rate = this->current_block->final_rate;
            }

        } else if (trapezoid_adjusted_rate != current_block->nominal_rate) {
            // If we are cruising
            // Make sure we cruise at exactly nominal rate
            this->trapezoid_adjusted_rate = this->current_block->nominal_rate;
        }

        if(last_rate != trapezoid_adjusted_rate) {
            // don't call this if speed did not change
            this->set_step_events_per_second(this->trapezoid_adjusted_rate);
        }
    }
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
inline void Stepper::trapezoid_generator_reset()
{
    this->trapezoid_adjusted_rate = this->current_block->initial_rate;
    this->force_speed_update = true;
}

// Update the speed for all steppers
void Stepper::set_step_events_per_second( float steps_per_second )
{
    float isps= steps_per_second / this->current_block->steps_event_count;

    // Instruct the stepper motors
    for (size_t i = 0; i < THEKERNEL->robot->actuators.size(); i++) {
        if (THEKERNEL->robot->actuators[i]->moving) {
            THEKERNEL->robot->actuators[i]->set_speed(isps * this->current_block->steps[i]);
        }
    }

    // Other modules might want to know the speed changed
    THEKERNEL->call_event(ON_SPEED_CHANGE, this);
}


