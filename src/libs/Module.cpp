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
    using std::placeholders::_1;
    switch(event_id) {
        case ON_MAIN_LOOP:              THEKERNEL->register_for_event(event_id, std::bind(&Module::on_main_loop,             this, _1)); break;
        case ON_CONSOLE_LINE_RECEIVED:  THEKERNEL->register_for_event(event_id, std::bind(&Module::on_console_line_received, this, _1)); break;
        case ON_GCODE_RECEIVED:         THEKERNEL->register_for_event(event_id, std::bind(&Module::on_gcode_received,        this, _1)); break;
        case ON_GCODE_EXECUTE:          THEKERNEL->register_for_event(event_id, std::bind(&Module::on_gcode_execute,         this, _1)); break;
        case ON_SPEED_CHANGE:           THEKERNEL->register_for_event(event_id, std::bind(&Module::on_speed_change,          this, _1)); break;
        case ON_BLOCK_BEGIN:            THEKERNEL->register_for_event(event_id, std::bind(&Module::on_block_begin,           this, _1)); break;
        case ON_BLOCK_END:              THEKERNEL->register_for_event(event_id, std::bind(&Module::on_block_end,             this, _1)); break;
        case ON_CONFIG_RELOAD:          THEKERNEL->register_for_event(event_id, std::bind(&Module::on_config_reload,         this, _1)); break;
        case ON_PLAY:                   THEKERNEL->register_for_event(event_id, std::bind(&Module::on_play,                  this, _1)); break;
        case ON_PAUSE:                  THEKERNEL->register_for_event(event_id, std::bind(&Module::on_pause,                 this, _1)); break;
        case ON_IDLE:                   THEKERNEL->register_for_event(event_id, std::bind(&Module::on_idle,                  this, _1)); break;
        case ON_CONFIG_VALUE:           THEKERNEL->register_for_event(event_id, std::bind(&Module::on_config_value,          this, _1)); break;
        case ON_CONFIG_COMPLETE:        THEKERNEL->register_for_event(event_id, std::bind(&Module::on_config_complete,       this, _1)); break;
        case ON_SECOND_TICK:            THEKERNEL->register_for_event(event_id, std::bind(&Module::on_second_tick,           this, _1)); break;
        case ON_GET_PUBLIC_DATA:        THEKERNEL->register_for_event(event_id, std::bind(&Module::on_get_public_data,       this, _1)); break;
        case ON_SET_PUBLIC_DATA:        THEKERNEL->register_for_event(event_id, std::bind(&Module::on_set_public_data,       this, _1)); break;
        case NUMBER_OF_DEFINED_EVENTS: break; // STFU
    }
}
