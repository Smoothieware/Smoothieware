/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "mbed.h"
#include "libs/Kernel.h"
#include "modules/tools/laser/Laser.h"
#include "modules/tools/extruder/Extruder.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/pauser/Pauser.h"
#include "libs/SDFileSystem.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

SDFileSystem sd(p5, p6, p7, p8, "sd");
//LocalFileSystem local("local");


int main() {

    Kernel* kernel = new Kernel();

    kernel->serial->printf("Smoothie ( grbl port ) version 0.2 \r\nstart\r\n");

    kernel->add_module( new Laser(p21) );
    kernel->add_module( new Extruder(p26) );
    kernel->add_module( new SimpleShell() );
    //kernel->add_module( new Pauser(p29,p30) );

    while(1){
        kernel->call_event(ON_MAIN_LOOP);
    }

}
