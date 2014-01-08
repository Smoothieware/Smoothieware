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
#include "libs/Pauser.h"
#include "libs/StreamOutputPool.h"
#include <mri.h>

#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Stepper.h"
#include "modules/robot/Conveyor.h"
#include "modules/tools/endstops/Endstops.h"
#include <malloc.h>



#define baud_rate_setting_checksum CHECKSUM("baud_rate")
#define uart0_checksum             CHECKSUM("uart0")

Kernel* Kernel::instance;

// This is used to configure UARTs depending on the MRI configuration, see Kernel::Kernel()
static int isDebugMonitorUsingUart0(){
    return NVIC_GetPriority(UART0_IRQn) == 0;
}

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel(){
    instance= this; // setup the Singleton instance of the kernel

    // Config first, because we need the baud_rate setting before we start serial
    this->config         = new Config();

    // Serial second, because the other modules might want to say something
    this->streams        = new StreamOutputPool();

    // Configure UART depending on MRI config
    // If MRI is using UART0, we want to use UART1, otherwise, we want to use UART0. This makes it easy to use only one UART for both debug and actual commands.
    NVIC_SetPriorityGrouping(0);
    if( !isDebugMonitorUsingUart0() ){
        this->serial         = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(9600)->as_number());
    }else{
        this->serial         = new SerialConsole(p13, p14, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(9600)->as_number());
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
    int base_stepping_frequency          =  this->config->value(base_stepping_frequency_checksum      )->by_default(100000)->as_number();
    float microseconds_per_step_pulse   =  this->config->value(microseconds_per_step_pulse_checksum  )->by_default(5     )->as_number();

    // Configure the step ticker ( TODO : shouldnt this go into stepticker's code ? )
    this->step_ticker->set_reset_delay( microseconds_per_step_pulse / 1000000L );
    this->step_ticker->set_frequency(   base_stepping_frequency );

    // Core modules
    this->add_module( this->gcode_dispatch = new GcodeDispatch() );
    this->add_module( this->robot          = new Robot()         );
    this->add_module( this->stepper        = new Stepper()       );
    this->add_module( this->planner        = new Planner()       );
    this->add_module( this->conveyor       = new Conveyor()      );
    this->add_module( this->pauser         = new Pauser()        );
    this->add_module( this->public_data    = new PublicData()    );
    this->add_module( this->toolsmanager   = new ToolsManager()    );

}

// Add a module to Kernel. We don't actually hold a list of modules, we just tell it where Kernel is
void Kernel::add_module(Module* module){
    module->on_module_loaded();
}

// Adds a hook for a given module and event
void Kernel::register_for_event(_EVENT_ENUM id_event, Module* module){
    this->hooks[id_event].push_back(module);
}

// Call a specific event without arguments
void Kernel::call_event(_EVENT_ENUM id_event){
    for (Module* current : hooks[id_event]) {
        (current->*kernel_callback_functions[id_event])(this);
    }
}

// Call a specific event with an argument
void Kernel::call_event(_EVENT_ENUM id_event, void * argument){
    for (Module* current : hooks[id_event]) {
        (current->*kernel_callback_functions[id_event])(argument);
    }
}
