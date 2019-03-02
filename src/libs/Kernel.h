/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KERNEL_H
#define KERNEL_H

#define THEKERNEL Kernel::instance
#define THECONVEYOR THEKERNEL->conveyor
#define THEROBOT THEKERNEL->robot

#include "Module.h"
#include <array>
#include <vector>
#include <string>

//Module manager
class Config;
class Module;
class Conveyor;
class SlowTicker;
class SerialConsole;
class StreamOutputPool;
class GcodeDispatch;
class Robot;
class Planner;
class StepTicker;
class Adc;
class PublicData;
class SimpleShell;
class Configurator;

class Kernel {
    public:
        Kernel();
        static Kernel* instance; // the Singleton instance of Kernel usable anywhere
        const char* config_override_filename(){ return "/sd/config-override"; }

        void add_module(Module* module);
        void register_for_event(_EVENT_ENUM id_event, Module *module);
        void call_event(_EVENT_ENUM id_event, void * argument= nullptr);

        bool kernel_has_event(_EVENT_ENUM id_event, Module *module);
        void unregister_for_event(_EVENT_ENUM id_event, Module *module);

        bool is_using_leds() const { return use_leds; }
        bool is_halted() const { return halted; }
        bool is_grbl_mode() const { return grbl_mode; }
        bool is_ok_per_line() const { return ok_per_line; }

        void set_feed_hold(bool f) { feed_hold= f; }
        bool get_feed_hold() const { return feed_hold; }
        bool is_feed_hold_enabled() const { return enable_feed_hold; }

        std::string get_query_string();

        // These modules are available to all other modules
        SerialConsole*    serial;
        StreamOutputPool* streams;
        GcodeDispatch*    gcode_dispatch;
        Robot*            robot;
        Planner*          planner;
        Config*           config;
        Conveyor*         conveyor;
        Configurator*     configurator;
        SimpleShell*      simpleshell;

        SlowTicker*       slow_ticker;
        StepTicker*       step_ticker;
        Adc*              adc;
        std::string       current_path;
        uint32_t          base_stepping_frequency;

    private:
        // When a module asks to be called for a specific event ( a hook ), this is where that request is remembered
        std::array<std::vector<Module*>, NUMBER_OF_DEFINED_EVENTS> hooks;
        struct {
            bool use_leds:1;
            bool halted:1;
            bool grbl_mode:1;
            bool feed_hold:1;
            bool ok_per_line:1;
            bool enable_feed_hold:1;
        };

};

#endif
