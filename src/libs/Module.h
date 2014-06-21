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
    #define EVENT(name, func) name,
    #include "Event.h"
    #undef EVENT
    NUMBER_OF_DEFINED_EVENTS
};

class Module;

typedef void (Module::*ModuleCallback)(void * argument);

extern const ModuleCallback kernel_callback_functions[NUMBER_OF_DEFINED_EVENTS];

// Module base class
// All modules must extend this class, see http://smoothieware.org/moduleexample
class Module {
    public:
        Module();
        virtual ~Module();
        virtual void on_module_loaded();
        void register_for_event(        _EVENT_ENUM event_id);
        #define EVENT(name, func) virtual void func (void*);
        #include "Event.h"
        #undef EVENT
};

#endif
