/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/extruder/ExtruderMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/touchprobe/Touchprobe.h"
#include "modules/tools/zprobe/ZProbe.h"
#include "modules/tools/switch/SwitchPool.h"

#include "modules/robot/Conveyor.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/pausebutton/PauseButton.h"
#include "modules/utils/PlayLed/PlayLed.h"
#include "modules/utils/panel/Panel.h"
#include "modules/utils/leds/Leds.h"
#include "libs/Network/uip/Network.h"
#include "PublicData.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

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

#include "mbed.h"

#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
#define disable_msd_checksum               CHECKSUM("msd_disable")

// Watchdog wd(5000000, WDT_MRI);

// USB Stuff
SDCard sd  __attribute__ ((section ("AHBSRAM0"))) (P0_9, P0_8, P0_7, P0_6);      // this selects SPI1 as the sdcard as it is on Smoothieboard
//SDCard sd(P0_18, P0_17, P0_15, P0_16);  // this selects SPI0 as the sdcard

USB u __attribute__ ((section ("AHBSRAM0")));
USBSerial usbserial __attribute__ ((section ("AHBSRAM0"))) (&u);
USBMSD msc __attribute__ ((section ("AHBSRAM0"))) (&u, &sd);
//USBMSD *msc= NULL;
DFU dfu __attribute__ ((section ("AHBSRAM0"))) (&u);

SDFAT mounter __attribute__ ((section ("AHBSRAM0"))) ("sd", &sd);

void init() {

    Kernel* kernel = new Kernel();

    kernel->streams->printf("Smoothie Running @%ldMHz\r\n", SystemCoreClock / 1000000);
    Version version;
    kernel->streams->printf("  Build version %s, Build date %s\r\n", version.get_build(), version.get_build_date());

    // attempt to be able to disable msd in config
    // if(!kernel->config->value( disable_msd_checksum )->by_default(false)->as_bool()){
    //     msc= new USBMSD(&u, &sd);
    // }else{
    //     msc= NULL;
    //     kernel->streams->printf("MSD is disabled\r\n");
    // }

    bool sdok= (sd.disk_initialize() == 0);

    // Create and add main modules
    kernel->add_module( new SimpleShell() );
    kernel->add_module( new Configurator() );


    // initialize usb/dfu/etc. first to allow flashing when a module loops endless
    // but doesn't help on exceptions

    // Create and initialize USB stuff
    u.init();
    //if(sdok) { // only do this if there is an sd disk
    //    msc= new USBMSD(&u, &sd);
    //    kernel->add_module( msc );
    //}

    // if(msc != NULL){
    //     kernel->add_module( msc );
    // }

    kernel->add_module( &msc );

    kernel->add_module( &usbserial );
    if( kernel->config->value( second_usb_serial_enable_checksum )->by_default(false)->as_bool() ){
        kernel->add_module( new USBSerial(&u) );
    }

    kernel->add_module( &dfu );
    kernel->add_module( &u );


    kernel->add_module( new Leds() );   // used below


    int post = 0;                       // after creating Leds module

    PublicData::set_value( leds_checksum, post_checksum, &(post=1));
    kernel->add_module( new CurrentControl() );

    PublicData::set_value( leds_checksum, post_checksum, &(post=2));
    kernel->add_module( new Endstops() );

    PublicData::set_value( leds_checksum, post_checksum, &(post=3));
    kernel->add_module( new Player() );

    PublicData::set_value( leds_checksum, post_checksum, &(post=4));
    kernel->add_module( new PauseButton() );

    PublicData::set_value( leds_checksum, post_checksum, &(post=5));
    kernel->add_module( new PlayLed() );


    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    // post starts at 8 (leds 00100) for these

    #ifndef NO_TOOLS_SWITCH
    PublicData::set_value( leds_checksum, post_checksum, &(post=8));
    SwitchPool *sp= new SwitchPool();
    sp->load_tools();
    delete sp;
    #endif
    #ifndef NO_TOOLS_EXTRUDER
    PublicData::set_value( leds_checksum, post_checksum, &(post=9));
    ExtruderMaker *em= new ExtruderMaker();
    em->load_tools();
    delete em;
    #endif
    #ifndef NO_TOOLS_TEMPERATURECONTROL
    PublicData::set_value( leds_checksum, post_checksum, &(post=10));
    // Note order is important here must be after extruder
    TemperatureControlPool *tp= new TemperatureControlPool();
    tp->load_tools();
    delete tp;
    #endif
    #ifndef NO_TOOLS_LASER
    PublicData::set_value( leds_checksum, post_checksum, &(post=11));
    kernel->add_module( new Laser() );
    #endif
    #ifndef NO_UTILS_PANEL
    PublicData::set_value( leds_checksum, post_checksum, &(post=12));
    kernel->add_module( new Panel() );
    #endif
    #ifndef NO_TOOLS_TOUCHPROBE
    PublicData::set_value( leds_checksum, post_checksum, &(post=13));
    kernel->add_module( new Touchprobe() );
    #endif
    #ifndef NO_TOOLS_ZPROBE
    PublicData::set_value( leds_checksum, post_checksum, &(post=14));
    kernel->add_module( new ZProbe() );
    #endif
    #ifndef NONETWORK
    PublicData::set_value( leds_checksum, post_checksum, &(post=15));
    kernel->add_module( new Network() );
    #endif

    PublicData::set_value( leds_checksum, post_checksum, &(post=END_OF_POST));

    // clear up the config cache to save some memory
    kernel->config->config_cache_clear();

    if(sdok) {
        PublicData::set_value( leds_checksum, sdok_checksum, 0 );

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
}

int main()
{
    init();

    // Main loop
    while(1){
        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
