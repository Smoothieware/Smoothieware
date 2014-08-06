/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
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
#include "modules/robot/Pauser.h"

#include <malloc.h>
#include <array>

#define baud_rate_setting_checksum CHECKSUM("baud_rate")
#define uart0_checksum             CHECKSUM("uart0")

#define base_stepping_frequency_checksum            CHECKSUM("base_stepping_frequency")
#define microseconds_per_step_pulse_checksum        CHECKSUM("microseconds_per_step_pulse")

Kernel* Kernel::instance;

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel(){
    instance= this; // setup the Singleton instance of the kernel

    // serial first at fixed baud rate (DEFAULT_SERIAL_BAUD_RATE) so config can report errors to serial
	// Set to UART0, this will be changed to use the same UART as MRI if it's enabled
    this->serial = new SerialConsole(USBTX, USBRX, DEFAULT_SERIAL_BAUD_RATE);

    // Config next, but does not load cache yet
    this->config         = new Config();

    // Pre-load the config cache, do after setting up serial so we can report errors to serial
    this->config->config_cache_load();

    // now config is loaded we can do normal setup for serial based on config
    delete this->serial;
    this->serial= NULL;

    this->streams        = new StreamOutputPool();

    this->current_path   = "/";

    // Configure UART depending on MRI config
    // Match up the SerialConsole to MRI UART. This makes it easy to use only one UART for both debug and actual commands.
    NVIC_SetPriorityGrouping(0);

#if MRI_ENABLE != 0
    switch( __mriPlatform_CommUartIndex() ) {
        case 0:
            this->serial = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 1:
            this->serial = new SerialConsole(  p13,   p14, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 2:
            this->serial = new SerialConsole(  p28,   p27, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 3:
            this->serial = new SerialConsole(   p9,   p10, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
    }
#endif
    // default
    if(this->serial == NULL) {
        this->serial = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
    }

    this->add_module( this->config );
    this->add_module( this->serial );

    // HAL stuff
    add_module( this->slow_ticker          = new SlowTicker());
    this->step_ticker          = new StepTicker();
    this->adc                  = new Adc();

    // TODO : These should go into platform-specific files
    // LPC17xx-specific
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_SetPriority(TIMER2_IRQn, 3);

    // Set other priorities lower than the timers
    NVIC_SetPriority(ADC_IRQn, 4);
    NVIC_SetPriority(USB_IRQn, 4);

    // If MRI is enabled
    if( MRI_ENABLE ){
        if( NVIC_GetPriority(UART0_IRQn) > 0 ){ NVIC_SetPriority(UART0_IRQn, 4); }
        if( NVIC_GetPriority(UART1_IRQn) > 0 ){ NVIC_SetPriority(UART1_IRQn, 4); }
        if( NVIC_GetPriority(UART2_IRQn) > 0 ){ NVIC_SetPriority(UART2_IRQn, 4); }
        if( NVIC_GetPriority(UART3_IRQn) > 0 ){ NVIC_SetPriority(UART3_IRQn, 4); }
    }else{
        NVIC_SetPriority(UART0_IRQn, 4);
        NVIC_SetPriority(UART1_IRQn, 4);
        NVIC_SetPriority(UART2_IRQn, 4);
        NVIC_SetPriority(UART3_IRQn, 4);
    }

    // Configure the step ticker
    this->base_stepping_frequency       =  this->config->value(base_stepping_frequency_checksum      )->by_default(100000)->as_number();
    float microseconds_per_step_pulse   =  this->config->value(microseconds_per_step_pulse_checksum  )->by_default(5     )->as_number();

    // Configure the step ticker ( TODO : shouldnt this go into stepticker's code ? )
    this->step_ticker->set_reset_delay( microseconds_per_step_pulse / 1000000L );
    this->step_ticker->set_frequency( this->base_stepping_frequency );

    // Core modules
    this->add_module( this->gcode_dispatch = new GcodeDispatch() );
    this->add_module( this->robot          = new Robot()         );
    this->add_module( this->stepper        = new Stepper()       );
    this->add_module( this->conveyor       = new Conveyor()      );
    this->add_module( this->pauser         = new Pauser()        );

    this->planner = new Planner();

}

// Add a module to Kernel. We don't actually hold a list of modules, we just tell it where Kernel is
void Kernel::add_module(Module* module){
    module->on_module_loaded();
}

// Adds a hook for a given module and event
void Kernel::register_for_event(_EVENT_ENUM id_event, Module *mod){
    this->hooks[id_event].push_back(mod);
}

// Call a specific event without arguments
void Kernel::call_event(_EVENT_ENUM id_event){
    for (auto m : hooks[id_event]) {
        (m->*kernel_callback_functions[id_event])(this);
    }
}

// Call a specific event with an argument
void Kernel::call_event(_EVENT_ENUM id_event, void * argument){
    for (auto m : hooks[id_event]) {
        (m->*kernel_callback_functions[id_event])(argument);
    }
}
