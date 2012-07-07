/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef KERNEL_H
#define KERNEL_H
#include "libs/Module.h"
#include "libs/Config.h"
#include "libs/SlowTicker.h"
#include "libs/StepTicker.h"
#include "libs/Adc.h"
#include "libs/Digipot.h"
#include "libs/Pauser.h"
#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Stepper.h"

// See : http://smoothieware.org/listofevents
#define NUMBER_OF_DEFINED_EVENTS   12
#define ON_MAIN_LOOP               0
#define ON_CONSOLE_LINE_RECEIVED   1
#define ON_GCODE_RECEIVED          2
#define ON_STEPPER_WAKE_UP         3    //TODO : Remove the need for this event, then this event itself eg: have planner call stepper directly
#define ON_GCODE_EXECUTE           4
#define ON_SPEED_CHANGE            5
#define ON_BLOCK_BEGIN             6
#define ON_BLOCK_END               7
#define ON_CONFIG_RELOAD           8
#define ON_PLAY                    9
#define ON_PAUSE                   10
#define ON_IDLE                    11


using namespace std;
#include <vector>

typedef void (Module::*ModuleCallback)(void * argument);

//Module manager
class Module;
class Player;
class SlowTicker;
class Kernel {
    public:
        Kernel();
        void add_module(Module* module);
        void register_for_event(unsigned int id_event, Module* module);
        void call_event(unsigned int id_event);
        void call_event(unsigned int id_event, void * argument);

        // These modules are aviable to all other modules
        SerialConsole*    serial;
        GcodeDispatch*    gcode_dispatch;
        Robot*            robot;
        Stepper*          stepper;
        Planner*          planner;
        Config*           config;
        Player*           player;
        Pauser*           pauser;

        int debug;
        SlowTicker*       slow_ticker;
        StepTicker*       step_ticker;
        Adc*              adc;
        Digipot*          digipot;

    private:
        vector<Module*> hooks[NUMBER_OF_DEFINED_EVENTS]; // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered

};

#endif
