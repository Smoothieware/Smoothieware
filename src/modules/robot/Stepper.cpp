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

#include <functional>

#include "libs/nuts_bolts.h"
#include "libs/Hook.h"

#include <mri.h>

// The stepper reacts to blocks that have XYZ movement to transform them into actual stepper motor moves
Stepper::Stepper()
{
    this->current_block = NULL;
}

//Called when the module has just been loaded
void Stepper::on_module_loaded()
{
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    // Get onfiguration
    this->on_config_reload(this);

    // Attach to the end_of_move stepper event
    THEKERNEL->step_ticker->finished_fnc= std::bind( &Stepper::stepper_motor_finished_move, this);
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
    }
}

void Stepper::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if( gcode->has_m) {
        if( gcode->m == 17 ) {
            this->turn_enable_pins_on();

        }else if( (gcode->m == 84 || gcode->m == 18) && !gcode->has_letter('E') ) {
            THEKERNEL->conveyor->wait_for_empty_queue();
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
    if(!take) return;

    block->take();

    // We can't move with the enable pins off
    if( this->enable_pins_status == false ) {
        this->turn_enable_pins_on();
    }

    this->current_block = block;

    // setup stepticker to execute this block
    // if it is running then add this to next block otherwise it needs to be started
    // if(!THEKERNEL->step_ticker->is_next_block()) {

    // }
    THEKERNEL->step_ticker->copy_block(block);
}

// Current block is discarded
void Stepper::on_block_end(void *argument)
{
    this->current_block = NULL; //stfu !
}

// When all moves in a block have finished this is called by step ticker (in the pendsv ISR)
void Stepper::stepper_motor_finished_move()
{
    // This block is finished, release it
    if( this->current_block != NULL ) {
        this->current_block->release();
    }
}
