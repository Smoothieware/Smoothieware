/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODULE_H
#define MODULE_H

#include <string>
using std::string;

// See : http://smoothieware.org/listofevents
enum _EVENT_ENUM {
    ON_MAIN_LOOP,
    ON_CONSOLE_LINE_RECEIVED,
    ON_GCODE_RECEIVED,
    ON_STEPPER_WAKE_UP,
    ON_GCODE_EXECUTE,
    ON_SPEED_CHANGE,
    ON_BLOCK_BEGIN,
    ON_BLOCK_END,
    ON_CONFIG_RELOAD,
    ON_PLAY,
    ON_PAUSE,
    ON_IDLE,
    ON_CONFIG_VALUE,
    ON_CONFIG_COMPLETE,
    ON_SECOND_TICK,
    NUMBER_OF_DEFINED_EVENTS
};

class Module;

typedef void (Module::*ModuleCallback)(void * argument);

extern const ModuleCallback kernel_callback_functions[NUMBER_OF_DEFINED_EVENTS];

// Module base class
// All modules must extend this class, see http://smoothieware.org/moduleexample
class Kernel;
class Module {
    public:
        Module();
        virtual void on_module_loaded();
        virtual void register_for_event(        _EVENT_ENUM event_id);
        virtual void on_main_loop(              void * argument);
        virtual void on_console_line_received(  void * argument);
        virtual void on_gcode_received(         void * argument);
        virtual void on_stepper_wake_up(        void * argument);
        virtual void on_gcode_execute(          void * argument);
        virtual void on_speed_change(           void * argument);
        virtual void on_block_begin(            void * argument);
        virtual void on_block_end(              void * argument);
        virtual void on_config_reload(          void * argument);
        virtual void on_play(                   void * argument);
        virtual void on_pause(                  void * argument);
        virtual void on_idle(                   void * argument);
        virtual void on_config_value(           void * argument);
        virtual void on_config_complete(        void * argument);
        virtual void on_second_tick(            void * argument);
        Kernel * kernel;
};

#endif
