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
#include "modules/robot/Conveyor.h"
#include "StepperMotor.h"
#include "BaseSolution.h"
#include "EndstopsPublicAccess.h"
#include "Configurator.h"
#include "SimpleShell.h"

#ifndef NO_TOOLS_LASER
#include "Laser.h"
#endif

#include "platform_memory.h"

#include <malloc.h>
#include <array>
#include <string>

#define laser_checksum CHECKSUM("laser")
#define baud_rate_setting_checksum CHECKSUM("baud_rate")
#define uart0_checksum             CHECKSUM("uart0")

#define base_stepping_frequency_checksum            CHECKSUM("base_stepping_frequency")
#define microseconds_per_step_pulse_checksum        CHECKSUM("microseconds_per_step_pulse")
#define disable_leds_checksum                       CHECKSUM("leds_disable")
#define grbl_mode_checksum                          CHECKSUM("grbl_mode")
#define feed_hold_enable_checksum                   CHECKSUM("enable_feed_hold")
#define ok_per_line_checksum                        CHECKSUM("ok_per_line")
#define new_status_format_checksum                  CHECKSUM("new_status_format")

Kernel* Kernel::instance;

// The kernel is the central point in Smoothie : it stores modules, and handles event calls
Kernel::Kernel()
{
    halted = false;
    feed_hold = false;
    enable_feed_hold = false;

    instance = this; // setup the Singleton instance of the kernel

    // serial first at fixed baud rate (DEFAULT_SERIAL_BAUD_RATE) so config can report errors to serial
    // Set to UART0, this will be changed to use the same UART as MRI if it's enabled
    this->serial = new SerialConsole(USBTX, USBRX, DEFAULT_SERIAL_BAUD_RATE);

    // Config next, but does not load cache yet
    this->config = new Config();

    // Pre-load the config cache, do after setting up serial so we can report errors to serial
    this->config->config_cache_load();

    // now config is loaded we can do normal setup for serial based on config
    delete this->serial;
    this->serial = NULL;

    this->streams = new StreamOutputPool();

    this->current_path   = "/";

    // Configure UART depending on MRI config
    // Match up the SerialConsole to MRI UART. This makes it easy to use only one UART for both debug and actual commands.
    NVIC_SetPriorityGrouping(0);

#if MRI_ENABLE != 0
    switch( __mriPlatform_CommUartIndex() ) {
        case 0:
            this->serial = new(AHB0) SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 1:
            this->serial = new(AHB0) SerialConsole(  p13,   p14, this->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 2:
            this->serial = new(AHB0) SerialConsole(  p28,   p27, this->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
        case 3:
            this->serial = new(AHB0) SerialConsole(   p9,   p10, this->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
            break;
    }
#endif
    // default
    if(this->serial == NULL) {
        this->serial = new(AHB0) SerialConsole(USBTX, USBRX, this->config->value(uart0_checksum, baud_rate_setting_checksum)->by_default(DEFAULT_SERIAL_BAUD_RATE)->as_number());
    }

    //some boards don't have leds.. TOO BAD!
    this->use_leds = !this->config->value( disable_leds_checksum )->by_default(false)->as_bool();

#ifdef CNC
    this->grbl_mode = this->config->value( grbl_mode_checksum )->by_default(true)->as_bool();
#else
    this->grbl_mode = this->config->value( grbl_mode_checksum )->by_default(false)->as_bool();
#endif

    this->enable_feed_hold = this->config->value( feed_hold_enable_checksum )->by_default(this->grbl_mode)->as_bool();

    // we expect ok per line now not per G code, setting this to false will return to the old (incorrect) way of ok per G code
    this->ok_per_line = this->config->value( ok_per_line_checksum )->by_default(true)->as_bool();

    this->new_status_format = this->config->value( new_status_format_checksum )->by_default(true)->as_bool();

    this->add_module( this->serial );

    // HAL stuff
    add_module( this->slow_ticker = new SlowTicker());

    this->step_ticker = new StepTicker();
    this->adc = new Adc();

    // TODO : These should go into platform-specific files
    // LPC17xx-specific
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_SetPriority(TIMER1_IRQn, 1);
    NVIC_SetPriority(TIMER2_IRQn, 4);
    NVIC_SetPriority(PendSV_IRQn, 3);

    // Set other priorities lower than the timers
    NVIC_SetPriority(ADC_IRQn, 5);
    NVIC_SetPriority(USB_IRQn, 5);

    // If MRI is enabled
    if( MRI_ENABLE ) {
        if( NVIC_GetPriority(UART0_IRQn) > 0 ) { NVIC_SetPriority(UART0_IRQn, 5); }
        if( NVIC_GetPriority(UART1_IRQn) > 0 ) { NVIC_SetPriority(UART1_IRQn, 5); }
        if( NVIC_GetPriority(UART2_IRQn) > 0 ) { NVIC_SetPriority(UART2_IRQn, 5); }
        if( NVIC_GetPriority(UART3_IRQn) > 0 ) { NVIC_SetPriority(UART3_IRQn, 5); }
    } else {
        NVIC_SetPriority(UART0_IRQn, 5);
        NVIC_SetPriority(UART1_IRQn, 5);
        NVIC_SetPriority(UART2_IRQn, 5);
        NVIC_SetPriority(UART3_IRQn, 5);
    }

    // Configure the step ticker
    this->base_stepping_frequency = this->config->value(base_stepping_frequency_checksum)->by_default(100000)->as_number();
    float microseconds_per_step_pulse = this->config->value(microseconds_per_step_pulse_checksum)->by_default(1)->as_number();

    // Configure the step ticker
    this->step_ticker->set_frequency( this->base_stepping_frequency );
    this->step_ticker->set_unstep_time( microseconds_per_step_pulse );

    // Core modules
    this->add_module( this->conveyor       = new Conveyor()      );
    this->add_module( this->gcode_dispatch = new GcodeDispatch() );
    this->add_module( this->robot          = new Robot()         );
    this->add_module( this->simpleshell    = new SimpleShell()   );

    this->planner = new Planner();
    this->configurator = new Configurator();
}

// return a GRBL-like query string for serial ?
std::string Kernel::get_query_string()
{
    std::string str;
    bool homing;
    bool ok = PublicData::get_value(endstops_checksum, get_homing_status_checksum, 0, &homing);
    if(!ok) homing = false;
    bool running = false;

    str.append("<");
    if(halted) {
        str.append("Alarm");
    } else if(homing) {
        running = true;
        str.append("Home");
    } else if(feed_hold) {
        str.append("Hold");
    } else if(this->conveyor->is_idle()) {
        str.append("Idle");
    } else {
        running = true;
        str.append("Run");
    }

    if(running) {
        float mpos[3];
        robot->get_current_machine_position(mpos);
        // current_position/mpos includes the compensation transform so we need to get the inverse to get actual position
        if(robot->compensationTransform) robot->compensationTransform(mpos, true); // get inverse compensation transform

        char buf[128];
        // machine position
        size_t n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(mpos[0]), robot->from_millimeters(mpos[1]), robot->from_millimeters(mpos[2]));
        if(n > sizeof(buf)) n= sizeof(buf);

        if(new_status_format) {
            str.append("|MPos:").append(buf, n);

#if MAX_ROBOT_ACTUATORS > 3
            // deal with the ABC axis (E will be A)
            for (int i = A_AXIS; i < robot->get_number_registered_motors(); ++i) {
                // current actuator position
                n = snprintf(buf, sizeof(buf), ",%1.4f", robot->from_millimeters(robot->actuators[i]->get_current_position()));
                if(n > sizeof(buf)) n= sizeof(buf);
                str.append(buf, n);
            }
#endif

        }else{
            str.append(",MPos:").append(buf, n);
        }

        // work space position
        Robot::wcs_t pos = robot->mcs2wcs(mpos);
        n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(pos)), robot->from_millimeters(std::get<Y_AXIS>(pos)), robot->from_millimeters(std::get<Z_AXIS>(pos)));
        if(n > sizeof(buf)) n= sizeof(buf);

        if(new_status_format) {
            str.append("|WPos:").append(buf, n);
            // current feedrate
            float fr= robot->from_millimeters(conveyor->get_current_feedrate()*60.0F);
            n = snprintf(buf, sizeof(buf), "|F:%1.4f", fr);
            if(n > sizeof(buf)) n= sizeof(buf);
            str.append(buf, n);
            float sr= robot->get_s_value();
            n = snprintf(buf, sizeof(buf), "|S:%1.4f", sr);
            if(n > sizeof(buf)) n= sizeof(buf);
            str.append(buf, n);

            // current Laser power
            #ifndef NO_TOOLS_LASER
                Laser *plaser= nullptr;
                if(PublicData::get_value(laser_checksum, (void *)&plaser) && plaser != nullptr) {
                   float lp= plaser->get_current_power();
                    n = snprintf(buf, sizeof(buf), "|L:%1.4f", lp);
                    if(n > sizeof(buf)) n= sizeof(buf);
                    str.append(buf, n);
                }
            #endif

        }else{
            str.append(",WPos:").append(buf, n);
        }

        str.append(">\r\n");

    } else {
        // return the last milestone if idle
        char buf[128];
        // machine position
        Robot::wcs_t mpos = robot->get_axis_position();
        size_t n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(mpos)), robot->from_millimeters(std::get<Y_AXIS>(mpos)), robot->from_millimeters(std::get<Z_AXIS>(mpos)));
        if(n > sizeof(buf)) n= sizeof(buf);
        if(new_status_format) {
            str.append("|MPos:").append(buf, n);
#if MAX_ROBOT_ACTUATORS > 3
            // deal with the ABC axis (E will be A)
            for (int i = A_AXIS; i < robot->get_number_registered_motors(); ++i) {
                // current actuator position
                n = snprintf(buf, sizeof(buf), ",%1.4f", robot->from_millimeters(robot->actuators[i]->get_current_position()));
                if(n > sizeof(buf)) n= sizeof(buf);
                str.append(buf, n);
            }
#endif

        }else{
            str.append(",MPos:").append(buf, n);
        }

        // work space position
        Robot::wcs_t pos = robot->mcs2wcs(mpos);
        n = snprintf(buf, sizeof(buf), "%1.4f,%1.4f,%1.4f", robot->from_millimeters(std::get<X_AXIS>(pos)), robot->from_millimeters(std::get<Y_AXIS>(pos)), robot->from_millimeters(std::get<Z_AXIS>(pos)));
        if(n > sizeof(buf)) n= sizeof(buf);
        if(new_status_format) {
            str.append("|WPos:").append(buf, n);
        }else{
            str.append(",WPos:").append(buf, n);
        }

        if(new_status_format) {
            float fr= robot->from_millimeters(robot->get_feed_rate());
            n = snprintf(buf, sizeof(buf), "|F:%1.4f", fr);
            if(n > sizeof(buf)) n= sizeof(buf);
            str.append(buf, n);
        }

        str.append(">\r\n");
    }
    return str;
}

// Add a module to Kernel. We don't actually hold a list of modules we just call its on_module_loaded
void Kernel::add_module(Module* module)
{
    module->on_module_loaded();
}

// Adds a hook for a given module and event
void Kernel::register_for_event(_EVENT_ENUM id_event, Module *mod)
{
    this->hooks[id_event].push_back(mod);
}

// Call a specific event with an argument
void Kernel::call_event(_EVENT_ENUM id_event, void * argument)
{
    bool was_idle = true;
    if(id_event == ON_HALT) {
        this->halted = (argument == nullptr);
        was_idle = conveyor->is_idle(); // see if we were doing anything like printing
    }

    // send to all registered modules
    for (auto m : hooks[id_event]) {
        (m->*kernel_callback_functions[id_event])(argument);
    }

    if(id_event == ON_HALT) {
        if(!this->halted || !was_idle) {
            // if we were running and this is a HALT
            // or if we are clearing the halt with $X or M999
            // fix up the current positions in case they got out of sync due to backed up commands
            this->robot->reset_position_from_current_actuator_position();
        }
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

