/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <string>
using std::string;
#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "utils/Gcode.h"
#include "libs/nuts_bolts.h"
#include "GcodeDispatch.h"

GcodeDispatch::GcodeDispatch(){}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded() {
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void * line){
    string possible_command = *static_cast<string*>(line);
    char first_char = possible_command[0];
    if( first_char == 'G' || first_char == 'M' || first_char == 'T' || first_char == 'S' ){ 
   
        //Remove comments
        size_t comment = possible_command.find_first_of(";");
        if( comment != string::npos ){ possible_command = possible_command.substr(0, comment); }

        // Dispatch
        Gcode gcode = Gcode();
        gcode.command = possible_command;
        this->kernel->call_event(ON_GCODE_RECEIVED, &gcode ); 
        this->kernel->serial->printf("ok\r\n");

    // Ignore comments 
    }else if( first_char == ';' || first_char == '(' ){
        this->kernel->serial->printf("ok\r\n");
    }
}

