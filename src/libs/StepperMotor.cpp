/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>

// in steps/sec the default minimum speed (was 20steps/sec hardcoded)
float StepperMotor::default_minimum_actuator_rate= 20.0F;

// A StepperMotor represents an actual stepper motor. It is used to generate steps that move the actual motor at a given speed

StepperMotor::StepperMotor()
{
    init();
}

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    init();
    enable(false);
    set_high_on_debug(en.port_number, en.pin);
}

StepperMotor::~StepperMotor()
{
}

void StepperMotor::init()
{
    // register this motor with the step ticker, and get its index in that array and bit position
    this->index= THEKERNEL->step_ticker->register_motor(this);
    this->moving = false;
    this->stepped = 0;
    this->is_move_finished = false;
    this->force_finish= false;

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
    current_position_steps= 0;
}

// // Instruct the StepperMotor to move a certain number of steps
// StepperMotor* StepperMotor::move( bool direction, unsigned int steps, float initial_speed)
// {
//     set_direction(direction);
//     this->direction = direction;
//     this->force_finish= false;

//     // Zero our tool counters
//     this->stepped = 0;
//     return this;
// }

void StepperMotor::change_steps_per_mm(float new_steps)
{
    steps_per_mm = new_steps;
    last_milestone_steps = lroundf(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

void StepperMotor::change_last_milestone(float new_milestone)
{
    last_milestone_mm = new_milestone;
    last_milestone_steps = lroundf(last_milestone_mm * steps_per_mm);
    current_position_steps = last_milestone_steps;
}

int  StepperMotor::steps_to_target(float target)
{
    int target_steps = lroundf(target * steps_per_mm);
    return target_steps - last_milestone_steps;
}
