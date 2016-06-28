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
#include "mbed.h"

StepperMotor::StepperMotor(Pin &step, Pin &dir, Pin &en) : step_pin(step), dir_pin(dir), en_pin(en)
{
    set_high_on_debug(en.port_number, en.pin);

    steps_per_mm         = 1.0F;
    max_rate             = 50.0F;

    last_milestone_steps = 0;
    last_milestone_mm    = 0.0F;
    current_position_steps= 0;
    enable(false);
    moving= false;
    acceleration= NAN;
    selected= true;

    this->register_for_event(ON_HALT);
    this->register_for_event(ON_ENABLE);
}

StepperMotor::~StepperMotor()
{
    THEKERNEL->unregister_for_event(ON_HALT, this);
    THEKERNEL->unregister_for_event(ON_ENABLE, this);
}

void StepperMotor::on_halt(void *argument)
{
    if(argument == nullptr) {
        enable(false);
        moving= false;
    }
}

void StepperMotor::on_enable(void *argument)
{
    enable(argument != nullptr);
}

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

void StepperMotor::set_last_milestones(float mm, int32_t steps)
{
    last_milestone_mm= mm;
    last_milestone_steps= steps;
    current_position_steps= last_milestone_steps;
}

void StepperMotor::update_last_milestones(float mm, int32_t steps)
{
    last_milestone_steps += steps;
    last_milestone_mm = mm;
}

int32_t StepperMotor::steps_to_target(float target)
{
    int32_t target_steps = lroundf(target * steps_per_mm);
    return target_steps - last_milestone_steps;
}

// Does a manual step pulse, used for direct encoder control of a stepper
void StepperMotor::manual_step(bool dir)
{
    if(!is_enabled()) enable(true);

    // set direction if needed
    if(this->direction != dir) {
        this->direction= dir;
        this->dir_pin.set(dir);
        wait_us(1);
    }

    // pulse step pin
    this->step_pin.set(1);
    wait_us(3);
    this->step_pin.set(0);


    // keep track of actuators actual position in steps
    this->current_position_steps += (dir ? -1 : 1);
}
