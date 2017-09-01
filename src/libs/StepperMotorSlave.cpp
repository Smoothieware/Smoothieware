/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "StepperMotorSlave.h"
#include "StepperMotor.h"

#include "Kernel.h"
#include "MRI_Hooks.h"
#include "StepTicker.h"

#include <math.h>
#include "mbed.h"

StepperMotorSlave::StepperMotorSlave(Pin &step, Pin &dir, Pin &en, Pin &slaveStep, Pin &slaveDir, Pin &slaveEn) : StepperMotor(step, dir, en),step_slave_pin(slaveStep), dir_slave_pin(slaveDir), en_slave_pin(slaveEn)
{

}

StepperMotorSlave::~StepperMotorSlave()
{
}


// Does a manual step pulse, used for direct encoder control of a stepper
// NOTE this is experimental and may change and/or be reomved in the future, it is an unsupported feature.
// use at your own risk
void StepperMotorSlave::manual_step(bool dir)//override of base function
{
    if(!is_enabled()) enable(true);

    // set direction if needed
    if(this->direction != dir) {
        this->direction= dir;
        this->dir_pin.set(dir);
        this->dir_slave_pin.set(dir);
        wait_us(1);
    }

    // pulse step pin
    this->step_pin.set(1);
	this->step_slave_pin.set(1);
    wait_us(3);
    this->step_pin.set(0);
	this->step_slave_pin.set(0);


    // keep track of actuators actual position in steps
    this->current_position_steps += (dir ? -1 : 1);
}
