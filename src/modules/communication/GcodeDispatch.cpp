/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <string>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "utils/Gcode.h"
#include "libs/nuts_bolts.h"
#include "GcodeDispatch.h"
#include "modules/robot/Player.h" 
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"

GcodeDispatch::GcodeDispatch(){}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded() {
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    currentline = -1;
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void * line){
    SerialMessage new_message = *static_cast<SerialMessage*>(line);
    string possible_command = new_message.message;    

    char first_char = possible_command[0];
    int ln = 0;
    int cs = 0;
    if( first_char == 'G' || first_char == 'M' || first_char == 'T' || first_char == 'N' ){ 

        //Get linenumber
        if( first_char == 'N' ){ 
            Gcode full_line = Gcode();
            full_line.command = possible_command;
            full_line.stream = new_message.stream;
            ln = (int) full_line.get_value('N');
            int chksum = (int) full_line.get_value('*');

            //Catch message if it is M110: Set Current Line Number
            if( full_line.has_letter('M') ){
                if( ((int) full_line.get_value('M')) == 110 ){
                    currentline = ln;
                    new_message.stream->printf("ok\r\n");
                    return;
                }
            }

            //Strip checksum value from possible_command
            size_t chkpos = possible_command.find_first_of("*");
            possible_command = possible_command.substr(0, chkpos); 
            //Calculate checksum
            if( chkpos != string::npos ){ 
                for(int i = 0; possible_command[i] != '*' && possible_command[i] != NULL; i++)
                    cs = cs ^ possible_command[i];
                cs &= 0xff;  // Defensive programming...
                cs -= chksum;
            }
            //Strip line number value from possible_command
            size_t lnsize = possible_command.find_first_of(" ") + 1;
            possible_command = possible_command.substr(lnsize); 

        }else{
            //Assume checks succeeded
            cs = 0x00;
            ln = currentline + 1;
        }

        //Remove comments
        size_t comment = possible_command.find_first_of(";");
        if( comment != string::npos ){ possible_command = possible_command.substr(0, comment); }

        //If checksum passes then process message, else request resend
        int nextline = currentline + 1;
        if( cs == 0x00 && ln == nextline ){
            if( first_char == 'N' ) {
                currentline = nextline;
            }

            while(possible_command.size() > 0) {
                size_t nextcmd = possible_command.find_first_of("GMT", possible_command.find_first_of("GMT")+1);
                string single_command;
                if(nextcmd == string::npos) {
                    single_command = possible_command;
                    possible_command = "";
                }
                else {
                    single_command = possible_command.substr(0,nextcmd);
                    possible_command = possible_command.substr(nextcmd);
                }
                //Prepare gcode for dispatch
                Gcode gcode = Gcode();
                gcode.command = single_command;
                gcode.stream = new_message.stream;

                //Dispatch message!
                this->kernel->call_event(ON_GCODE_RECEIVED, &gcode );
                new_message.stream->printf("ok\r\n");
            }
        }else{
            //Request resend
            new_message.stream->printf("rs N%d\r\n", nextline);
        }

    // Ignore comments and blank lines
    }else if( first_char == ';' || first_char == '(' || first_char == ' ' || first_char == '\n' || first_char == '\r' ){
        new_message.stream->printf("ok\r\n");
    }
}

