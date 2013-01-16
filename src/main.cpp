/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "modules/tools/laser/Laser.h"
#include "modules/tools/extruder/Extruder.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/robot/Player.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/pausebutton/PauseButton.h"
// #include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/Config.h"
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

#include "libs/Watchdog.h"

// Watchdog wd(5000000, WDT_MRI);

// #include "libs/USBCDCMSC/USBCDCMSC.h"
// SDFileSystem sd(p5, p6, p7, p8, "sd");  // LPC17xx specific : comment if you are not using a SD card ( for example with a mBed ).
SDCard sd(P0_9, P0_8, P0_7, P0_6);
//LocalFileSystem local("local");       // LPC17xx specific : comment if you are not running a mBed
// USBCDCMSC cdcmsc(&sd);                  // LPC17xx specific : Composite serial + msc USB device

USB u;

USBSerial usbserial(&u);
USBMSD msc(&u, &sd);
DFU dfu(&u);
USBSerial usbserial2(&u);

SDFAT mounter("sd", &sd);

char buf[512];

GPIO leds[5] = {
    GPIO(P1_18),
    GPIO(P1_19),
    GPIO(P1_20),
    GPIO(P1_21),
    GPIO(P4_28)
};

int main() {
    for (int i = 0; i < 5; i++)
    {
        leds[i].output();
        leds[i] = (i & 1) ^ 1;
    }

    sd.disk_initialize();

    Kernel* kernel = new Kernel();

    kernel->streams->printf("Smoothie ( grbl port ) version 0.7.2 \r\n");

//     kernel->streams->printf("Disk Status: %d, Type: %d\n", sd.disk_status(), sd.card_type());
//     if (sd.disk_status() == 0) {
//         uint16_t s1;
//         uint8_t s2;
//         char suffix;
//         if (sd.disk_sectors() >= (1<<21)) {
//             s1 = sd.disk_sectors() >> 21;
//             s2 = ((sd.disk_sectors() * 10) >> 21) - (s1 * 10);
//             suffix = 'G';
//         }
//         else if (sd.disk_sectors() >= (1<<11)) {
//             s1 = sd.disk_sectors() >> 11;
//             s2 = ((sd.disk_sectors() * 10) >> 11) - (s1 * 10);
//             suffix = 'M';
//         }
//         else if (sd.disk_sectors() >= (1<< 1)) {
//             s1 = sd.disk_sectors() >> 1;
//             s2 = ((sd.disk_sectors() * 10) >> 1) - (s1 * 10);
//             suffix = 'K';
//         }
//         else {
//             s1 = sd.disk_sectors() << 9;
//             s2 = 0;
//             suffix = ' ';
//         }
//         kernel->streams->printf("Card has %lu blocks; %llu bytes; %d.%d%cB\n", sd.disk_sectors(), sd.disk_size(), s1, s2, suffix);
//     }

//     kernel->add_module( new Laser(p21) );
    kernel->add_module( new Extruder() );
    kernel->add_module( new SimpleShell() );
    kernel->add_module( new Configurator() );
    kernel->add_module( new CurrentControl() );
    kernel->add_module( new TemperatureControlPool() );
    kernel->add_module( new SwitchPool() );
    kernel->add_module( new PauseButton() );
    kernel->add_module( new Endstops() );

    u.init();

    kernel->add_module( &msc );
    kernel->add_module( &usbserial );
    kernel->add_module( &usbserial2 );
    kernel->add_module( &dfu );
    kernel->add_module( &u );

    struct SerialMessage message;
    message.message = "G90";
    message.stream = kernel->serial;
    kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

    while(1){
        kernel->call_event(ON_MAIN_LOOP);
        kernel->call_event(ON_IDLE);
    }
}
