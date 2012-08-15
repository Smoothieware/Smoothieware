/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Kernel.h"
#include "modules/tools/laser/Laser.h"
//#include "modules/tools/extruder/Extruder.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/robot/Player.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/pausebutton/PauseButton.h"
#include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

#include "libs/USBCDCMSC/USBCDCMSC.h"
SDFileSystem sd(p5, p6, p7, p8, "sd");  // LPC17xx specific : comment if you are not using a SD card ( for example with a mBed ).
//LocalFileSystem local("local");       // LPC17xx specific : comment if you are not running a mBed
USBCDCMSC cdcmsc(&sd);                  // LPC17xx specific : Composite serial + msc USB device

int main() {
    
    Kernel* kernel = new Kernel();

    kernel->streams->printf("Smoothie ( grbl port ) version 0.6.1 \r\n");

    kernel->add_module( new Laser(p21) );
    //kernel->add_module( new Extruder() );
    kernel->add_module( new SimpleShell() );
    kernel->add_module( new Configurator() );
    kernel->add_module( new CurrentControl() );
    kernel->add_module( new TemperatureControlPool() );
    kernel->add_module( new SwitchPool() );
    kernel->add_module( new PauseButton() );   
    kernel->add_module( new Endstops() );

    kernel->add_module( &cdcmsc );
   
    kernel->streams->printf("start\r\n");

    while(1){
        kernel->call_event(ON_MAIN_LOOP);
        kernel->call_event(ON_IDLE);
    }
}

