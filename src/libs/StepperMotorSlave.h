/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "StepperMotor.h"

class StepperMotorSlave  : public StepperMotor{
    public:
        StepperMotorSlave(Pin &step, Pin &dir, Pin &en, Pin &slaveStep, Pin &slaveDir, Pin &slaveEn) ;
        ~StepperMotorSlave();

        virtual inline bool step() { step_pin.set(1);step_slave_pin.set(1); current_position_steps += (direction?-1:1); return moving; }//override
        // called from unstep ISR
        virtual inline void unstep() { step_pin.set(0); step_slave_pin.set(0);}//override
        // called from step ticker ISR
        virtual inline void set_direction(bool f) { dir_pin.set(f);//function override
		                                    dir_slave_pin.set(f);
		                                    direction= f; }

        virtual void enable(bool state) { en_pin.set(!state);//function override
								  en_slave_pin.set(!state);
								};
								
		virtual void manual_step(bool dir);//override of base function
    private:
  
		Pin step_slave_pin;
        Pin dir_slave_pin;
        Pin en_slave_pin;

};

