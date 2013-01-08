/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

// #include <string>
// using std::string;

#include <cstring>

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "utils/Gcode.h"
#include "libs/nuts_bolts.h"
#include "GcodeDispatch.h"
#include "modules/robot/Player.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"

#include "utils.h"

GcodeDispatch::GcodeDispatch(){}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded() {
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    currentline = -1;
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void * line){
    SerialMessage new_message = *static_cast<SerialMessage*>(line);
    char* possible_command = strdup(new_message.message);

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
            int chkpos = find_first_of(possible_command, '*');
//             possible_command = possible_command.substr(0, chkpos);
            //Calculate checksum
            if( chkpos >= 0 ){
                possible_command[chkpos] = 0;
                for(int i = 0; possible_command[i] != '*' && possible_command[i] != 0; i++)
                    cs = cs ^ possible_command[i];
                cs &= 0xff;  // Defensive programming...
                cs -= chksum;
            }
            //Strip line number value from possible_command
//             size_t lnsize = possible_command.find_first_of(" ") + 1;
//             possible_command = possible_command.substr(lnsize);
            int lnsize = find_first_of(possible_command, 'N');
            if (lnsize >= 0)
            {
                char* n = strdup(possible_command + lnsize + find_first_of(possible_command, ' '));
//                 possible_command += lnsize;
//                 possible_command += find_first_of(possible_command, ' ');
                free(possible_command);
                possible_command = n;
            }

        }else{
            //Assume checks succeeded
            cs = 0x00;
            ln = currentline + 1;
        }

        //Remove comments
//         size_t comment = possible_command.find_first_of(";");
//         if( comment != string::npos ){ possible_command = possible_command.substr(0, comment); }
        int comment;
        comment = find_first_of(possible_command, ';');
        if (comment >= 0)
            possible_command[comment] = 0;
        comment = find_first_of(possible_command, '(');
        if (comment >= 0)
            possible_command[comment] = 0;

        //If checksum passes then process message, else request resend
        int nextline = currentline + 1;
        if( cs == 0x00 && ln == nextline ){
            if( first_char == 'N' ) {
                currentline = nextline;
            }

            while(strlen((char*) possible_command) > 0) {
                size_t nextcmd = find_first_of(possible_command + find_first_of(possible_command, "GMT")+1, "GMT");
                char* single_command;
                if(nextcmd == string::npos) {
                    single_command = strdup(possible_command);
                    possible_command[0] = 0;
                }
                else {
//                     single_command = possible_command.substr(0,nextcmd);
                    single_command = strndup(possible_command, nextcmd);
//                     possible_command = possible_command.substr(nextcmd);
                    char* n = strdup(possible_command + nextcmd);
                    free(possible_command);
                    possible_command = n;
                }
                //Prepare gcode for dispatch
                Gcode gcode = Gcode();
                gcode.command = single_command;
                gcode.stream = new_message.stream;
                gcode.prepare_cached_values();

                //Dispatch message!
                gcode.add_nl = false;
                this->kernel->call_event(ON_GCODE_RECEIVED, &gcode );
                if (gcode.add_nl)
                    new_message.stream->printf("\r\n");
                new_message.stream->printf("ok\r\n");
                free(single_command);
            }
        }else{
            //Request resend
            new_message.stream->printf("rs N%d\r\n", nextline);
        }

    // Ignore comments and blank lines
    }else if( first_char == ';' || first_char == '(' || first_char == ' ' || first_char == '\n' || first_char == '\r' ){
        new_message.stream->printf("ok\r\n");
    }

    free(possible_command);
}

