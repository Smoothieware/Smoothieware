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
#include "libs/Digipot.h"
#include "libs/Pauser.h"
#include "libs/StreamOutputPool.h"
#include <mri.h>

#include "modules/communication/SerialConsole.h"
#include "modules/communication/GcodeDispatch.h"
#include "modules/robot/Planner.h"
#include "modules/robot/Robot.h"
#include "modules/robot/Stepper.h"
#include "modules/robot/Player.h"
#include <malloc.h>



// List of callback functions, ordered as their corresponding events
const ModuleCallback kernel_callback_functions[NUMBER_OF_DEFINED_EVENTS] = {
        &Module::on_main_loop,
        &Module::on_console_line_received,
        &Module::on_gcode_received,
        &Module::on_stepper_wake_up,
        &Module::on_gcode_execute,
        &Module::on_speed_change,
        &Module::on_block_begin,
        &Module::on_block_end,
        &Module::on_config_reload,
        &Module::on_play,
        &Module::on_pause,
        &Module::on_idle
};

#define baud_rate_setting_checksum 10922
#define uart0_checksum             16877

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel(){
    extern SerialConsole uart;

    uart.printf("Kernel: ");

    // Value init for the arrays
    for( uint8_t i=0; i<NUMBER_OF_DEFINED_EVENTS; i++ ){
        for( uint8_t index=0; index<32; index++ ){
            this->hooks[i][index] = NULL;
        }
    }

    uart.printf("\t[new Config]\n");

    // Config first, because we need the baud_rate setting before we start serial
    this->config         = new Config();

    uart.printf("\t[new StreamOutputPool]\n");

    // Serial second, because the other modules might want to say something
    this->streams        = new StreamOutputPool();

    // Configure UART depending on MRI config
    NVIC_SetPriorityGrouping(0);
    /*
    if (strstr(MRI_UART, "MRI_UART_MBED_USB")){
        if (strstr(MRI_UART, "MRI_UART_SHARED")){
            this->serial         = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(9600)->as_number());
        }else{
            this->serial         = new SerialConsole(p13, p14, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(9600)->as_number());
        }
    }else{
        this->serial         = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(9600)->as_number());
    }
    */
//     if( NVIC_GetPriority(UART0_IRQn) > 0 ){
//         this->serial         = new SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(2000000)->as_number());
//     }else{
//         this->serial         = new SerialConsole(p13, p14, this->config->value(uart0_checksum,baud_rate_setting_checksum)->by_default(2000000)->as_number());
//     }
    this->serial = &uart;

    uart.printf("\tADD config\n");

    this->add_module( this->config );

//     __debugbreak();

    uart.printf("\tADD serial\n");
    this->add_module( this->serial );

    // HAL stuff
    uart.printf("\tADD SlowTicker\n");
    this->slow_ticker          = new SlowTicker();
    uart.printf("\tADD StepTicker\n");
    this->step_ticker          = new StepTicker();
    uart.printf("\tADD Adc\n");
    this->adc                  = new Adc();
    uart.printf("\tADD Digipot\n");
    this->digipot              = new Digipot();

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

//     uart.printf("Configure:\n\tStepTicker:\n");
    // Configure the step ticker
    int base_stepping_frequency       =  this->config->value(base_stepping_frequency_checksum      )->by_default(100000)->as_number();
    double microseconds_per_step_pulse   =  this->config->value(microseconds_per_step_pulse_checksum  )->by_default(5     )->as_number();

//     uart.printf("\t\tset_reset_delay: %u\n", microseconds_per_step_pulse / 1000000L);
//     this->step_ticker->set_reset_delay( microseconds_per_step_pulse / 1000000L );

//     uart.printf("\t\tset_frequency: %u\n", base_stepping_frequency);
//     this->step_ticker->set_frequency(   base_stepping_frequency );

    // Core modules
    uart.printf("\tADD GcodeDispatch\n");
    this->add_module( this->gcode_dispatch = new GcodeDispatch() );
    uart.printf("\tADD Robot\n");
    this->add_module( this->robot          = new Robot()         );
    uart.printf("\tADD Stepper\n");
    this->add_module( this->stepper        = new Stepper()       );
    uart.printf("\tADD Planner\n");
    this->add_module( this->planner        = new Planner()       );
    uart.printf("\tADD Player\n");
    this->add_module( this->player         = new Player()        );
    uart.printf("\tADD Pauser\n");
    this->add_module( this->pauser         = new Pauser()        );

    uart.printf("Kernel Complete!\n");
}

void Kernel::add_module(Module* module){
    module->kernel = this;
    module->on_module_loaded();
    module->register_for_event(ON_CONFIG_RELOAD);
}

void Kernel::register_for_event(unsigned int id_event, Module* module){
    uint8_t current_id = 0;
    Module* current = this->hooks[id_event][0];
    while(current != NULL ){
        if( current == module ){ return; }
        current_id++;
        current = this->hooks[id_event][current_id];
    }
    this->hooks[id_event][current_id] = module;
    this->hooks[id_event][current_id+1] = NULL;
}

void Kernel::call_event(unsigned int id_event){
    uint8_t current_id = 0; Module* current = this->hooks[id_event][0];
    while(current != NULL ){   // For each active stepper
        (current->*kernel_callback_functions[id_event])(this);
        current_id++;
        current = this->hooks[id_event][current_id];
    }
}

void Kernel::call_event(unsigned int id_event, void * argument){
    uint8_t current_id = 0; Module* current = this->hooks[id_event][0];
    while(current != NULL ){   // For each active stepper
        (current->*kernel_callback_functions[id_event])(argument);
        current_id++;
        current = this->hooks[id_event][current_id];
    }
}
