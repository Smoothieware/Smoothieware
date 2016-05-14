/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_H
#define STEPPER_H

#include "libs/Module.h"
#include <stdint.h>

class Block;
class StepperMotor;

class Stepper : public Module
{
public:
    Stepper();
    void on_module_loaded();
    void on_config_reload(void *argument);
    void on_block_begin(void *argument);
    void on_block_end(void *argument);
    void on_gcode_received(void *argument);
    void on_halt(void *argument);

    void turn_enable_pins_on();
    void turn_enable_pins_off();

    const Block *get_current_block() const { return current_block; }

private:
    void stepper_motor_finished_move();

    Block *current_block;

    struct {
        bool enable_pins_status:1;
    };

};




#endif
