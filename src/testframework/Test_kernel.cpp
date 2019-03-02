/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/**
This is aprt of the Smoothie test framework, it generates a Mockable Kernl so kernel calls can be tested for
*/

#include "libs/Kernel.h"
#include "libs/Module.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "libs/SlowTicker.h"
#include "libs/Adc.h"
#include "libs/StreamOutputPool.h"
#include <mri.h>
#include "checksumm.h"
#include "ConfigValue.h"

#include "libs/StepTicker.h"
#include "libs/PublicData.h"
#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Stepper.h"
#include "modules/robot/Conveyor.h"

#include "Config.h"
#include "FirmConfigSource.h"

#include <malloc.h>
#include <array>
#include <functional>
#include <map>

Kernel* Kernel::instance;

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel(){
    instance= this; // setup the Singleton instance of the kernel

    // serial first at fixed baud rate (DEFAULT_SERIAL_BAUD_RATE) so config can report errors to serial
    // Set to UART0, this will be changed to use the same UART as MRI if it's enabled
    this->serial = new SerialConsole(USBTX, USBRX, DEFAULT_SERIAL_BAUD_RATE);

    // Config next, but does not load cache yet
    // loads config from in memory source for test framework must be loaded by test
    this->config = nullptr;

    this->streams = new StreamOutputPool();
    this->streams->append_stream(this->serial);

    this->current_path   = "/";

    this->slow_ticker = new SlowTicker();

    // dummies (would be nice to refactor to not have to create a conveyor)
    this->conveyor= new Conveyor();

    // Configure UART depending on MRI config
    // Match up the SerialConsole to MRI UART. This makes it easy to use only one UART for both debug and actual commands.
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(UART0_IRQn, 5);
}

// Add a module to Kernel. We don't actually hold a list of modules we just call its on_module_loaded
void Kernel::add_module(Module* module){
    module->on_module_loaded();
}

// Adds a hook for a given module and event
void Kernel::register_for_event(_EVENT_ENUM id_event, Module *mod){
    this->hooks[id_event].push_back(mod);
}

static std::map<_EVENT_ENUM, std::function<void(void*)> > event_callbacks;

// Call a specific event with an argument
void Kernel::call_event(_EVENT_ENUM id_event, void * argument){
    for (auto m : hooks[id_event]) {
        (m->*kernel_callback_functions[id_event])(argument);
    }
    if(event_callbacks.find(id_event) != event_callbacks.end()){
        event_callbacks[id_event](argument);
    }else{
        printf("call_event for event: %d not handled\n", id_event);
    }
}

// These are used by tests to test for various things. basically mocks
bool Kernel::kernel_has_event(_EVENT_ENUM id_event, Module *mod)
{
    for (auto m : hooks[id_event]) {
        if(m == mod) return true;
    }
    return false;
}

void Kernel::unregister_for_event(_EVENT_ENUM id_event, Module *mod)
{
    for (auto i = hooks[id_event].begin(); i != hooks[id_event].end(); ++i) {
        if(*i == mod) {
            hooks[id_event].erase(i);
            return;
        }
    }
}

void test_kernel_setup_config(const char* start, const char* end)
{
    THEKERNEL->config= new Config(new FirmConfigSource("rom", start, end) );
    // Pre-load the config cache
    THEKERNEL->config->config_cache_load();
}

void test_kernel_teardown()
{
    delete THEKERNEL->config;
    THEKERNEL->config= nullptr;
    event_callbacks.clear();
}

void test_kernel_trap_event(_EVENT_ENUM id_event, std::function<void(void*)> fnc)
{
    event_callbacks[id_event]= fnc;
}

void test_kernel_untrap_event(_EVENT_ENUM id_event)
{
    event_callbacks.erase(id_event);
}
