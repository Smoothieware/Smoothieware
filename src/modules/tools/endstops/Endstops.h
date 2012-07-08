/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef ENDSTOPS_MODULE_H
#define ENDSTOPS_MODULE_H

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "libs/StepperMotor.h"

#define ALPHA_AXIS 0
#define BETA_AXIS  1
#define GAMMA_AXIS 2

#define NOT_HOMING 0
#define MOVING_TO_ORIGIN_FAST 1

class Endstops : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);

        StepperMotor* steppers[3];
};

















#endif
