/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/spindle/SpindleMaker.h"
#include "modules/tools/extruder/ExtruderMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/zprobe/ZProbe.h"
#include "modules/tools/scaracal/SCARAcal.h"
#include "RotaryDeltaCalibration.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/tools/temperatureswitch/TemperatureSwitch.h"
#include "modules/tools/drillingcycles/Drillingcycles.h"
#include "FilamentDetector.h"
#include "MotorDriverControl.h"

#include "modules/robot/Conveyor.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/killbutton/KillButton.h"
#include "modules/utils/PlayLed/PlayLed.h"
#include "modules/utils/panel/Panel.h"
#include "libs/Network/uip/Network.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "SlowTicker.h"
#include "Robot.h"

// #include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

// Debug
#include "libs/SerialMessage.h"

#include "libs/USBDevice/USB.h"
#include "libs/USBDevice/USBMSD/USBMSD.h"
#include "libs/USBDevice/USBMSD/SDCard.h"
#include "libs/USBDevice/USBSerial/USBSerial.h"
#include "libs/USBDevice/DFU.h"
#include "libs/SDFAT.h"
#include "StreamOutputPool.h"
#include "ToolManager.h"

#include "libs/Watchdog.h"

#include "version.h"
#include "system_LPC17xx.h"
#include "platform_memory.h"

#include "mbed.h"

#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
#define disable_msd_checksum  CHECKSUM("msd_disable")
#define dfu_enable_checksum  CHECKSUM("dfu_enable")
#define watchdog_timeout_checksum  CHECKSUM("watchdog_timeout")


// USB Stuff
SDCard sd  __attribute__ ((section ("AHBSRAM0"))) (P0_9, P0_8, P0_7, P0_6);      // this selects SPI1 as the sdcard as it is on Smoothieboard
//SDCard sd(P0_18, P0_17, P0_15, P0_16);  // this selects SPI0 as the sdcard
//SDCard sd(P0_18, P0_17, P0_15, P2_8);  // this selects SPI0 as the sdcard witrh a different sd select

USB u __attribute__ ((section ("AHBSRAM0")));
USBSerial usbserial __attribute__ ((section ("AHBSRAM0"))) (&u);
#ifndef DISABLEMSD
USBMSD msc __attribute__ ((section ("AHBSRAM0"))) (&u, &sd);
#else
USBMSD *msc= NULL;
#endif

SDFAT mounter __attribute__ ((section ("AHBSRAM0"))) ("sd", &sd);

GPIO leds[5] = {
    GPIO(P1_18),
    GPIO(P1_19),
    GPIO(P1_20),
    GPIO(P1_21),
    GPIO(P4_28)
};

void init() {

    // Default pins to low status
    for (int i = 0; i < 5; i++){
        leds[i].output();
        leds[i]= 0;
    }

    Kernel* kernel = new Kernel();

    kernel->streams->printf("Smoothie Running @%ldMHz\r\n", SystemCoreClock / 1000000);
    SimpleShell::version_command("", kernel->streams);

    bool sdok= (sd.disk_initialize() == 0);
    if(!sdok) kernel->streams->printf("SDCard failed to initialize\r\n");

    #ifdef NONETWORK
        kernel->streams->printf("NETWORK is disabled\r\n");
    #endif

#ifdef DISABLEMSD
    // attempt to be able to disable msd in config
    if(sdok && !kernel->config->value( disable_msd_checksum )->by_default(true)->as_bool()){
        // HACK to zero the memory USBMSD uses as it and its objects seem to not initialize properly in the ctor
        size_t n= sizeof(USBMSD);
        void *v = AHB0.alloc(n);
        memset(v, 0, n); // clear the allocated memory
        msc= new(v) USBMSD(&u, &sd); // allocate object using zeroed memory
    }else{
        msc= NULL;
        kernel->streams->printf("MSD is disabled\r\n");
    }
#endif

    // Create and add main modules
    kernel->add_module( new(AHB0) Player() );

    kernel->add_module( new(AHB0) CurrentControl() );
    kernel->add_module( new(AHB0) KillButton() );
    kernel->add_module( new(AHB0) PlayLed() );

    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    #ifndef NO_TOOLS_SWITCH
    SwitchPool *sp= new SwitchPool();
    sp->load_tools();
    delete sp;
    #endif
    #ifndef NO_TOOLS_EXTRUDER
    // NOTE this must be done first before Temperature control so ToolManager can handle Tn before temperaturecontrol module does
    ExtruderMaker *em= new ExtruderMaker();
    em->load_tools();
    delete em;
    #endif
    #ifndef NO_TOOLS_TEMPERATURECONTROL
    // Note order is important here must be after extruder so Tn as a parameter will get executed first
    TemperatureControlPool *tp= new TemperatureControlPool();
    tp->load_tools();
    delete tp;
    #endif
    #ifndef NO_TOOLS_ENDSTOPS
    kernel->add_module( new(AHB0) Endstops() );
    #endif
    #ifndef NO_TOOLS_LASER
    kernel->add_module( new Laser() );
    #endif
    #ifndef NO_TOOLS_SPINDLE
    SpindleMaker *sm= new SpindleMaker();
    sm->load_spindle();
    delete sm;
    //kernel->add_module( new(AHB0) Spindle() );
    #endif
    #ifndef NO_UTILS_PANEL
    kernel->add_module( new(AHB0) Panel() );
    #endif
    #ifndef NO_TOOLS_ZPROBE
    kernel->add_module( new(AHB0) ZProbe() );
    #endif
    #ifndef NO_TOOLS_SCARACAL
    kernel->add_module( new(AHB0) SCARAcal() );
    #endif
    #ifndef NO_TOOLS_ROTARYDELTACALIBRATION
    kernel->add_module( new(AHB0) RotaryDeltaCalibration() );
    #endif
    #ifndef NONETWORK
    kernel->add_module( new Network() );
    #endif
    #ifndef NO_TOOLS_TEMPERATURESWITCH
    // Must be loaded after TemperatureControl
    kernel->add_module( new(AHB0) TemperatureSwitch() );
    #endif
    #ifndef NO_TOOLS_DRILLINGCYCLES
    kernel->add_module( new(AHB0) Drillingcycles() );
    #endif
    #ifndef NO_TOOLS_FILAMENTDETECTOR
    kernel->add_module( new(AHB0) FilamentDetector() );
    #endif
    #ifndef NO_UTILS_MOTORDRIVERCONTROL
    kernel->add_module( new MotorDriverControl(0) );
    #endif
    // Create and initialize USB stuff
    u.init();

#ifdef DISABLEMSD
    if(sdok && msc != NULL){
        kernel->add_module( msc );
    }
#else
    kernel->add_module( &msc );
#endif

    kernel->add_module( &usbserial );
    if( kernel->config->value( second_usb_serial_enable_checksum )->by_default(false)->as_bool() ){
        kernel->add_module( new(AHB0) USBSerial(&u) );
    }

    if( kernel->config->value( dfu_enable_checksum )->by_default(false)->as_bool() ){
        kernel->add_module( new(AHB0) DFU(&u));
    }

    // 10 second watchdog timeout (or config as seconds)
    float t= kernel->config->value( watchdog_timeout_checksum )->by_default(10.0F)->as_number();
    if(t > 0.1F) {
        // NOTE setting WDT_RESET with the current bootloader would leave it in DFU mode which would be suboptimal
        kernel->add_module( new Watchdog(t*1000000, WDT_MRI)); // WDT_RESET));
        kernel->streams->printf("Watchdog enabled for %f seconds\n", t);
    }else{
        kernel->streams->printf("WARNING Watchdog is disabled\n");
    }


    kernel->add_module( &u );

    // memory before cache is cleared
    //SimpleShell::print_mem(kernel->streams);

    // clear up the config cache to save some memory
    kernel->config->config_cache_clear();

    if(kernel->is_using_leds()) {
        // set some leds to indicate status... led0 init done, led1 mainloop running, led2 idle loop running, led3 sdcard ok
        leds[0]= 1; // indicate we are done with init
        leds[3]= sdok?1:0; // 4th led indicates sdcard is available (TODO maye should indicate config was found)
    }

    if(sdok) {
        // load config override file if present
        // NOTE only Mxxx commands that set values should be put in this file. The file is generated by M500
        FILE *fp= fopen(kernel->config_override_filename(), "r");
        if(fp != NULL) {
            char buf[132];
            kernel->streams->printf("Loading config override file: %s...\n", kernel->config_override_filename());
            while(fgets(buf, sizeof buf, fp) != NULL) {
                kernel->streams->printf("  %s", buf);
                if(buf[0] == ';') continue; // skip the comments
                struct SerialMessage message= {&(StreamOutput::NullStream), buf};
                kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            }
            kernel->streams->printf("config override file executed\n");
            fclose(fp);
        }
    }

    // start the timers and interrupts
    THEKERNEL->conveyor->start(THEROBOT->get_number_registered_motors());
    THEKERNEL->step_ticker->start();
    THEKERNEL->slow_ticker->start();
}

int main()
{
    init();

    uint16_t cnt= 0;
    // Main loop
    while(1){
        if(THEKERNEL->is_using_leds()) {
            // flash led 2 to show we are alive
            leds[1]= (cnt++ & 0x1000) ? 1 : 0;
        }
        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
