/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODULE_H
#define MODULE_H

// See : http://smoothieware.org/listofevents
// When adding a new event the virtual method needs to be defined in class Module and the method pointer need to be defined in
// Module.cpp:16 in the same order
enum _EVENT_ENUM {
    ON_MAIN_LOOP,
    ON_CONSOLE_LINE_RECEIVED,
    ON_GCODE_RECEIVED,
    ON_GCODE_EXECUTE,
    ON_SPEED_CHANGE,
    ON_BLOCK_BEGIN,
    ON_BLOCK_END,
    ON_PLAY,
    ON_PAUSE,
    ON_IDLE,
    ON_SECOND_TICK,
    ON_GET_PUBLIC_DATA,
    ON_SET_PUBLIC_DATA,
    ON_HALT,
    NUMBER_OF_DEFINED_EVENTS
};

class Module;
typedef void (Module::*ModuleCallback)(void *argument);
extern const ModuleCallback kernel_callback_functions[NUMBER_OF_DEFINED_EVENTS];

// Module base class
// All modules must extend this class, see http://smoothieware.org/moduleexample
class Module
{
public:
    Module();
    virtual ~Module();
    virtual void on_module_loaded() {};

    void register_for_event(_EVENT_ENUM event_id);

    // event callbacks, not every module will implement all of these
    // there should be one for each _EVENT_ENUM
    virtual void on_main_loop(void *) {};
    virtual void on_console_line_received(void *) {};
    virtual void on_gcode_received(void *) {};
    virtual void on_gcode_execute(void *) {};
    virtual void on_speed_change(void *) {};
    virtual void on_block_begin(void *) {};
    virtual void on_block_end(void *) {};
    virtual void on_play(void *) {};
    virtual void on_pause(void *) {};
    virtual void on_idle(void *) {};
    virtual void on_second_tick(void *) {};
    virtual void on_get_public_data(void *) {};
    virtual void on_set_public_data(void *) {};
    virtual void on_halt(void *) {};

};

#endif
