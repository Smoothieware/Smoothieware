/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KERNEL_H
#define KERNEL_H

#define THEKERNEL Kernel::instance

#include "Module.h"
#include <array>
#include <vector>
#include <string>

//Module manager
class Config;
class Module;
class Conveyor;
class SlowTicker;
class Pauser;
class SerialConsole;
class StreamOutputPool;
class GcodeDispatch;
class Robot;
class Stepper;
class Planner;
class StepTicker;
class Adc;
class PublicData;
class TemperatureControlPool;

class Kernel {
    public:
        Kernel();
        static Kernel* instance; // the Singleton instance of Kernel usable anywhere
        const char* config_override_filename(){ return "/sd/config-override"; }

        void add_module(Module* module);
        void register_for_event(_EVENT_ENUM id_event, Module *module);
        void call_event(_EVENT_ENUM id_event);
        void call_event(_EVENT_ENUM id_event, void * argument);

        // These modules are available to all other modules
        SerialConsole*    serial;
        StreamOutputPool* streams;

        Robot*            robot;
        Stepper*          stepper;
        Planner*          planner;
        Config*           config;
        Conveyor*         conveyor;
        Pauser*           pauser;
        TemperatureControlPool* temperature_control_pool;

        int debug;
        SlowTicker*       slow_ticker;
        StepTicker*       step_ticker;
        Adc*              adc;
        bool              use_leds;
        std::string       current_path;
        uint32_t          base_stepping_frequency;
        uint32_t          acceleration_ticks_per_second;

    private:
        // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered
        std::array<std::vector<Module*>, NUMBER_OF_DEFINED_EVENTS> hooks;

};

#endif
