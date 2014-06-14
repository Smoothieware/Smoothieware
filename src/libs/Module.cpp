/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

Module::Module(){}
Module::~Module(){}

void Module::register_for_event(_EVENT_ENUM event_id){
    // Events are the basic building blocks of Smoothie. They register for events, and then do stuff when those events are called.
    // You add things to Smoothie by making a new class that inherits the Module class. See http://smoothieware.org/moduleexample for a crude introduction
    THEKERNEL->register_for_event(event_id, this);
}

void Module::call_event(_EVENT_ENUM event_id, void *arg)
{
    switch(event_id) {
        case ON_MAIN_LOOP:              this->on_main_loop(arg); break;
        case ON_CONSOLE_LINE_RECEIVED:  this->on_console_line_received(arg); break;
        case ON_GCODE_RECEIVED:         this->on_gcode_received(arg); break;
        case ON_GCODE_EXECUTE:          this->on_gcode_execute(arg); break;
        case ON_SPEED_CHANGE:           this->on_speed_change(arg); break;
        case ON_BLOCK_BEGIN:            this->on_block_begin(arg); break;
        case ON_BLOCK_END:              this->on_block_end(arg); break;
        case ON_CONFIG_RELOAD:          this->on_config_reload(arg); break;
        case ON_PLAY:                   this->on_play(arg); break;
        case ON_PAUSE:                  this->on_pause(arg); break;
        case ON_IDLE:                   this->on_idle(arg); break;
        case ON_CONFIG_VALUE:           this->on_config_value(arg); break;
        case ON_CONFIG_COMPLETE:        this->on_config_complete(arg); break;
        case ON_SECOND_TICK:            this->on_second_tick(arg); break;
        case ON_GET_PUBLIC_DATA:        this->on_get_public_data(arg); break;
        case ON_SET_PUBLIC_DATA:        this->on_set_public_data(arg)    ; break;
        case NUMBER_OF_DEFINED_EVENTS: break; // STFU
    }
}
