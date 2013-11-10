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
#include "libs/StreamOutputPool.h"
#include "libs/StepTicker.h"
#include "libs/Adc.h"
#include "libs/Pauser.h"
#include "libs/PublicData.h"
#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Stepper.h"
#include <array>

#define THEKERNEL Kernel::instance

//Module manager
class Config;
class Module;
class Conveyor;
class SlowTicker;
class Kernel {
    public:
        Kernel();
        static Kernel* instance; // the Singleton instance of Kernel usable anywhere

        void add_module(Module* module);
        void register_for_event(_EVENT_ENUM id_event, Module* module);
        void call_event(_EVENT_ENUM id_event);
        void call_event(_EVENT_ENUM id_event, void * argument);

        // These modules are aviable to all other modules
        SerialConsole*    serial;
        StreamOutputPool* streams;

        GcodeDispatch*    gcode_dispatch;
        Robot*            robot;
        Stepper*          stepper;
        Planner*          planner;
        Config*           config;
        Conveyor*         conveyor;
        Pauser*           pauser;

        int debug;
        SlowTicker*       slow_ticker;
        StepTicker*       step_ticker;
        Adc*              adc;
        PublicData*       public_data;

    private:
        std::array<std::vector<Module*>, NUMBER_OF_DEFINED_EVENTS> hooks; // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered

};

#endif
