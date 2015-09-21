/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "GcodeDispatch.h"

#include "libs/Kernel.h"
#include "utils/Gcode.h"
#include "Pauser.h"
#include "libs/nuts_bolts.h"
#include "modules/robot/Conveyor.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "libs/FileStream.h"
#include "libs/AppendFileStream.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "SimpleShell.h"
#include "utils.h"

#define return_error_on_unhandled_gcode_checksum    CHECKSUM("return_error_on_unhandled_gcode")
#define panel_display_message_checksum CHECKSUM("display_message")
#define panel_checksum             CHECKSUM("panel")

// goes in Flash, list of Mxxx codes that are allowed when in Halted state
static const int allowed_mcodes[]= {105,114}; // get temp, get pos
static bool is_allowed_mcode(int m) {
    for (size_t i = 0; i < sizeof(allowed_mcodes)/sizeof(int); ++i) {
        if(allowed_mcodes[i] == m) return true;
    }
    return false;
}

GcodeDispatch::GcodeDispatch()
{
    halted= false;
    uploading = false;
    currentline = -1;
    last_g= 255;
}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_HALT);
}

void GcodeDispatch::on_halt(void *arg)
{
    // set halt stream and ignore everything until M999
    this->halted= (arg == nullptr);
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void *line)
{
    SerialMessage new_message = *static_cast<SerialMessage *>(line);
    string possible_command = new_message.message;

    int ln = 0;
    int cs = 0;

try_again:

    char first_char = possible_command[0];
    unsigned int n;
    if ( first_char == 'G' || first_char == 'M' || first_char == 'T' || first_char == 'N' ) {

        //Get linenumber
        if ( first_char == 'N' ) {
            Gcode full_line = Gcode(possible_command, new_message.stream, false);
            ln = (int) full_line.get_value('N');
            int chksum = (int) full_line.get_value('*');

            //Catch message if it is M110: Set Current Line Number
            if ( full_line.has_m ) {
                if ( full_line.m == 110 ) {
                    currentline = ln;
                    new_message.stream->printf("ok\r\n");
                    return;
                }
            }

            //Strip checksum value from possible_command
            size_t chkpos = possible_command.find_first_of("*");
            possible_command = possible_command.substr(0, chkpos);
            //Calculate checksum
            if ( chkpos != string::npos ) {
                for (auto c = possible_command.cbegin(); *c != '*' && c != possible_command.cend(); c++)
                    cs = cs ^ *c;
                cs &= 0xff;  // Defensive programming...
                cs -= chksum;
            }
            //Strip line number value from possible_command
            size_t lnsize = possible_command.find_first_not_of("N0123456789.,- ");
            possible_command = possible_command.substr(lnsize);

        } else {
            //Assume checks succeeded
            cs = 0x00;
            ln = currentline + 1;
        }

        //Remove comments
        size_t comment = possible_command.find_first_of(";(");
        if( comment != string::npos ) {
            possible_command = possible_command.substr(0, comment);
        }

        //If checksum passes then process message, else request resend
        int nextline = currentline + 1;
        if( cs == 0x00 && ln == nextline ) {
            if( first_char == 'N' ) {
                currentline = nextline;
            }

            while(possible_command.size() > 0) {
                // assumes G or M are always the first on the line
                size_t nextcmd = possible_command.find_first_of("GM", 2);
                string single_command;
                if(nextcmd == string::npos) {
                    single_command = possible_command;
                    possible_command = "";
                } else {
                    single_command = possible_command.substr(0, nextcmd);
                    possible_command = possible_command.substr(nextcmd);
                }


                if(!uploading) {
                    //Prepare gcode for dispatch
                    Gcode *gcode = new Gcode(single_command, new_message.stream);

                    if(halted) {
                        // we ignore all commands until M999, unless it is in the exceptions list (like M105 get temp)
                        if(gcode->has_m && gcode->m == 999) {
                            THEKERNEL->call_event(ON_HALT, (void *)1); // clears on_halt
                            halted= false;
                            // fall through and pass onto other modules

                        }else if(!is_allowed_mcode(gcode->m)) {
                            // ignore everything, return error string to host
                            new_message.stream->printf("!!\r\n");
                            delete gcode;
                            continue;
                        }
                    }

                    if(gcode->has_g) {
                        last_g= gcode->g;
                    }

                    if(gcode->has_m) {
                        switch (gcode->m) {
                            case 28: // start upload command
                                delete gcode;

                                this->upload_filename = "/sd/" + single_command.substr(4); // rest of line is filename
                                // open file
                                upload_fd = fopen(this->upload_filename.c_str(), "w");
                                if(upload_fd != NULL) {
                                    this->uploading = true;
                                    new_message.stream->printf("Writing to file: %s\r\nok\r\n", this->upload_filename.c_str());
                                } else {
                                    new_message.stream->printf("open failed, File: %s.\r\nok\r\n", this->upload_filename.c_str());
                                }
                                //printf("Start Uploading file: %s, %p\n", upload_filename.c_str(), upload_fd);
                                continue;

                            case 112: // emergency stop, do the best we can with this
                                // TODO this really needs to be handled out-of-band
                                // disables heaters and motors, ignores further incoming Gcode and clears block queue
                                THEKERNEL->call_event(ON_HALT, nullptr);
                                THEKERNEL->streams->printf("ok Emergency Stop Requested - reset or M999 required to continue\r\n");
                                delete gcode;
                                return;

                            case 117: // M117 is a special non compliant Gcode as it allows arbitrary text on the line following the command
                            {    // concatenate the command again and send to panel if enabled
                                string str= single_command.substr(4) + possible_command;
                                PublicData::set_value( panel_checksum, panel_display_message_checksum, &str );
                                delete gcode;
                                new_message.stream->printf("ok\r\n");
                                return;
                            }

                            case 1000: // M1000 is a special comanad that will pass thru the raw lowercased command to the simpleshell (for hosts that do not allow such things)
                            {
                                // reconstruct entire command line again
                                string str= single_command.substr(5) + possible_command;
                                while(is_whitespace(str.front())){ str= str.substr(1); } // strip leading whitespace

                                delete gcode;

                                if(str.empty()) {
                                    SimpleShell::parse_command("help", "", new_message.stream);

                                }else{
                                    string args= lc(str);
                                    string cmd = shift_parameter(args);
                                    // find command and execute it
                                    if(!SimpleShell::parse_command(cmd.c_str(), args, new_message.stream)) {
                                        new_message.stream->printf("Command not found: %s\n", cmd.c_str());
                                    }
                                }

                                new_message.stream->printf("ok\r\n");
                                return;
                            }

                            case 500: // M500 save volatile settings to config-override
                                THEKERNEL->conveyor->wait_for_empty_queue(); //just to be safe as it can take a while to run
                                //remove(THEKERNEL->config_override_filename()); // seems to cause a hang every now and then
                                {
                                    FileStream fs(THEKERNEL->config_override_filename());
                                    fs.printf("; DO NOT EDIT THIS FILE\n");
                                    // this also will truncate the existing file instead of deleting it
                                }
                                // replace stream with one that writes to config-override file
                                gcode->stream = new AppendFileStream(THEKERNEL->config_override_filename());
                                // dispatch the M500 here so we can free up the stream when done
                                THEKERNEL->call_event(ON_GCODE_RECEIVED, gcode );
                                delete gcode->stream;
                                delete gcode;
                                new_message.stream->printf("Settings Stored to %s\r\nok\r\n", THEKERNEL->config_override_filename());
                                continue;

                            case 502: // M502 deletes config-override so everything defaults to what is in config
                                remove(THEKERNEL->config_override_filename());
                                new_message.stream->printf("config override file deleted %s, reboot needed\r\nok\r\n", THEKERNEL->config_override_filename());
                                delete gcode;
                                continue;

                            case 503: { // M503 display live settings and indicates if there is an override file
                                FILE *fd = fopen(THEKERNEL->config_override_filename(), "r");
                                if(fd != NULL) {
                                    fclose(fd);
                                    new_message.stream->printf("; config override present: %s\n",  THEKERNEL->config_override_filename());

                                } else {
                                    new_message.stream->printf("; No config override\n");
                                }
                                gcode->add_nl= true;
                                break; // fall through to process by modules
                            }
                        }
                    }

                    //printf("dispatch %p: '%s' G%d M%d...", gcode, gcode->command.c_str(), gcode->g, gcode->m);
                    //Dispatch message!
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, gcode );
                    if(gcode->add_nl)
                        new_message.stream->printf("\r\n");

                    if(!gcode->txt_after_ok.empty()) {
                        new_message.stream->printf("ok %s\r\n", gcode->txt_after_ok.c_str());
                        gcode->txt_after_ok.clear();
                    } else
                        new_message.stream->printf("ok\r\n");

                    delete gcode;

                } else {
                    // we are uploading a file so save it
                    if(single_command.substr(0, 3) == "M29") {
                        // done uploading, close file
                        fclose(upload_fd);
                        upload_fd = NULL;
                        uploading = false;
                        upload_filename.clear();
                        new_message.stream->printf("Done saving file.\r\nok\r\n");
                        continue;
                    }

                    if(upload_fd == NULL) {
                        // error detected writing to file so discard everything until it stops
                        new_message.stream->printf("ok\r\n");
                        continue;
                    }

                    single_command.append("\n");
                    static int cnt = 0;
                    if(fwrite(single_command.c_str(), 1, single_command.size(), upload_fd) != single_command.size()) {
                        // error writing to file
                        new_message.stream->printf("Error:error writing to file.\r\n");
                        fclose(upload_fd);
                        upload_fd = NULL;
                        continue;

                    } else {
                        cnt += single_command.size();
                        if (cnt > 400) {
                            // HACK ALERT to get around fwrite corruption close and re open for append
                            fclose(upload_fd);
                            upload_fd = fopen(upload_filename.c_str(), "a");
                            cnt = 0;
                        }
                        new_message.stream->printf("ok\r\n");
                        //printf("uploading file write ok\n");
                    }
                }
            }

        } else {
            //Request resend
            new_message.stream->printf("rs N%d\r\n", nextline);
        }

    } else if( (n=possible_command.find_first_of("XYZF")) == 0 || (first_char == ' ' && n != string::npos) ) {
        // handle pycam syntax, use last G0 or G1 and resubmit if an X Y Z or F is found on its own line
        if(last_g != 0 && last_g != 1) {
            //if no last G1 or G0 ignore
            //THEKERNEL->streams->printf("ignored: %s\r\n", possible_command.c_str());
            return;
        }
        char buf[6];
        snprintf(buf, sizeof(buf), "G%d ", last_g);
        possible_command.insert(0, buf);
        goto try_again;

        // Ignore comments and blank lines
    } else if ( first_char == ';' || first_char == '(' || first_char == ' ' || first_char == '\n' || first_char == '\r' ) {
        new_message.stream->printf("ok\r\n");
    }
}

