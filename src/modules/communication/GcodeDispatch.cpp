/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "GcodeDispatch.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "utils/Gcode.h"
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
#include "LPC17xx.h"
#include "version.h"

#define panel_display_message_checksum CHECKSUM("display_message")
#define panel_checksum             CHECKSUM("panel")

// goes in Flash, list of Mxxx codes that are allowed when in Halted state
static const int allowed_mcodes[]= {2,5,9,30,105,114,119,80,81,911,503,106,107}; // get temp, get pos, get endstops etc
static bool is_allowed_mcode(int m) {
    for (size_t i = 0; i < sizeof(allowed_mcodes)/sizeof(int); ++i) {
        if(allowed_mcodes[i] == m) return true;
    }
    return false;
}

GcodeDispatch::GcodeDispatch()
{
    uploading = false;
    currentline = -1;
    modal_group_1= 0;
}

// Called when the module has just been loaded
void GcodeDispatch::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
}

// When a command is received, if it is a Gcode, dispatch it as an object via an event
void GcodeDispatch::on_console_line_received(void *line)
{
    SerialMessage new_message = *static_cast<SerialMessage *>(line);
    string possible_command = new_message.message;

    int ln = 0;
    int cs = 0;

    // just reply ok to empty lines
    if(possible_command.empty()) {
        new_message.stream->printf("ok\r\n");
        return;
    }

try_again:

    char first_char = possible_command[0];
    unsigned int n;

    if(first_char == '$') {
        // ignore as simpleshell will handle it
        return;

    }else if(islower(first_char)) {
        // ignore all lowercase as they are simpleshell commands
        return;
    }

    if ( first_char == 'G' || first_char == 'M' || first_char == 'T' || first_char == 'S' || first_char == 'N' ) {

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

			//Calculate checksum
            if ( chkpos != string::npos ) {
				possible_command = possible_command.substr(0, chkpos);
                for (auto c = possible_command.cbegin(); *c != '*' && c != possible_command.cend(); c++)
                    cs = cs ^ *c;
                cs &= 0xff;  // Defensive programming...
                cs -= chksum;
			}

            //Strip line number value from possible_command
			size_t lnsize = possible_command.find_first_not_of("N0123456789.,- ");
			if(lnsize != string::npos) {
				possible_command = possible_command.substr(lnsize);
			}else{
				// it is a blank line
				possible_command.clear();
			}

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

            bool sent_ok= false; // used for G1 optimization
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


                if(!uploading || upload_stream != new_message.stream) {
                    // Prepare gcode for dispatch
                    Gcode *gcode = new Gcode(single_command, new_message.stream);

                    if(THEKERNEL->is_halted()) {
                        // we ignore all commands until M999, unless it is in the exceptions list (like M105 get temp)
                        if(gcode->has_m && gcode->m == 999) {
                            if(THEKERNEL->is_halted()) {
                                THEKERNEL->call_event(ON_HALT, (void *)1); // clears on_halt
                                new_message.stream->printf("WARNING: After HALT you should HOME as position is currently unknown\n");
                            }
                            new_message.stream->printf("ok\n");
                            delete gcode;
                            return;

                        }else if(!is_allowed_mcode(gcode->m)) {
                            // ignore everything, return error string to host
                            if(THEKERNEL->is_grbl_mode()) {
                                new_message.stream->printf("error:Alarm lock\n");

                            }else{
                                new_message.stream->printf("!!\r\n");
                            }
                            delete gcode;
                            return;
                        }
                    }

                    if(gcode->has_g) {
                        if(gcode->g == 53) { // G53 makes next movement command use machine coordinates
                            // this is ugly to implement as there may or may not be a G0/G1 on the same line
                            // valid version seem to include G53 G0 X1 Y2 Z3 G53 X1 Y2
                            if(possible_command.empty()) {
                                // use last gcode G1 or G0 if none on the line, and pass through as if it was a G0/G1
                                // TODO it is really an error if the last is not G0 thru G3
                                if(modal_group_1 > 3) {
                                    delete gcode;
                                    new_message.stream->printf("ok - Invalid G53\r\n");
                                    return;
                                }
                                // use last G0 or G1
                                gcode->g= modal_group_1;

                            }else{
                                delete gcode;
                                // extract next G0/G1 from the rest of the line, ignore if it is not one of these
                                gcode = new Gcode(possible_command, new_message.stream);
                                possible_command= "";
                                if(!gcode->has_g || gcode->g > 1) {
                                    // not G0 or G1 so ignore it as it is invalid
                                    delete gcode;
                                    new_message.stream->printf("ok - Invalid G53\r\n");
                                    return;
                                }
                            }
                            // makes it handle the parameters as a machine position
                            THEROBOT->next_command_is_MCS= true;

                        } else if(gcode->g == 1) {
                            // optimize G1 to send ok immediately (one per line) before it is planned
                            if(!sent_ok) {
                                sent_ok= true;
                                new_message.stream->printf("ok\n");
                            }
                        }

                        // remember last modal group 1 code
                        if(gcode->g < 4) {
                            modal_group_1= gcode->g;
                        }
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

                                // only save stuff from this stream
                                upload_stream= new_message.stream;

                                //printf("Start Uploading file: %s, %p\n", upload_filename.c_str(), upload_fd);
                                continue;

                            case 30: // end of program
                                if(!THEKERNEL->is_grbl_mode()) break; // Special case M30 as it is also delete sd card file so only do this if in grbl mode
                                // fall through to M2
                            case 2:
                                {
                                    modal_group_1= 1; // set to G1
                                    // issue M5 and M9 in case spindle and coolant are being used
                                    Gcode gc1("M5", &StreamOutput::NullStream);
                                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc1);
                                    Gcode gc2("M9", &StreamOutput::NullStream);
                                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);
                                }
                                break;

                            case 112: // emergency stop, do the best we can with this
                                // this is also handled out-of-band (it is now with ^X in the serial driver)
                                // disables heaters and motors, ignores further incoming Gcode and clears block queue
                                THEKERNEL->call_event(ON_HALT, nullptr);
                                THEKERNEL->streams->printf("ok Emergency Stop Requested - reset or M999 required to exit HALT state\r\n");
                                delete gcode;
                                return;

                            case 115: { // M115 Get firmware version and capabilities
                                Version vers;

                                new_message.stream->printf("FIRMWARE_NAME:Smoothieware, FIRMWARE_URL:http%%3A//smoothieware.org, X-SOURCE_CODE_URL:https://github.com/Smoothieware/Smoothieware, FIRMWARE_VERSION:%s, X-FIRMWARE_BUILD_DATE:%s, X-SYSTEM_CLOCK:%ldMHz, X-AXES:%d, X-GRBL_MODE:%d", vers.get_build(), vers.get_build_date(), SystemCoreClock / 1000000, MAX_ROBOT_ACTUATORS, THEKERNEL->is_grbl_mode());

                                #ifdef CNC
                                new_message.stream->printf(", X-CNC:1");
                                #else
                                new_message.stream->printf(", X-CNC:0");
                                #endif

                                #ifdef DISABLEMSD
                                new_message.stream->printf(", X-MSD:0");
                                #else
                                new_message.stream->printf(", X-MSD:1");
                                #endif

                                new_message.stream->printf("\nok\n");
                                return;
                            }

                            case 117: // M117 is a special non compliant Gcode as it allows arbitrary text on the line following the command
                            {    // concatenate the command again and send to panel if enabled
                                string str= single_command.substr(4) + possible_command;
                                PublicData::set_value( panel_checksum, panel_display_message_checksum, &str );
                                delete gcode;
                                new_message.stream->printf("ok\r\n");
                                return;
                            }

                            case 1000: // M1000 is a special command that will pass thru the raw lowercased command to the simpleshell (for hosts that do not allow such things)
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
                                THEKERNEL->conveyor->wait_for_idle(); //just to be safe as it can take a while to run
                                //remove(THEKERNEL->config_override_filename()); // seems to cause a hang every now and then
                                __disable_irq();
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
                                __enable_irq();
                                new_message.stream->printf("Settings Stored to %s\r\nok\r\n", THEKERNEL->config_override_filename());
                                continue;

                            case 501: // load config override
                            case 504: // save to specific config override file
                                {
                                    string arg= get_arguments(single_command + possible_command); // rest of line is filename
                                    if(arg.empty()) arg= "/sd/config-override";
                                    else arg= "/sd/config-override." + arg;
                                    //new_message.stream->printf("args: <%s>\n", arg.c_str());
                                    SimpleShell::parse_command((gcode->m == 501) ? "load_command" : "save_command", arg, new_message.stream);
                                }
                                delete gcode;
                                new_message.stream->printf("ok\r\n");
                                return;

                            case 502: // M502 deletes config-override so everything defaults to what is in config
                                remove(THEKERNEL->config_override_filename());
                                delete gcode;
                                new_message.stream->printf("config override file deleted %s, reboot needed\r\nok\r\n", THEKERNEL->config_override_filename());
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

                    if (gcode->is_error) {
                        // report error
                        if(THEKERNEL->is_grbl_mode()) {
                            new_message.stream->printf("error:");
                        }else{
                            new_message.stream->printf("Error: ");
                        }

                        if(!gcode->txt_after_ok.empty()) {
                            new_message.stream->printf("%s\r\n", gcode->txt_after_ok.c_str());
                            gcode->txt_after_ok.clear();

                        }else{
                            new_message.stream->printf("unknown\r\n");
                        }

                        // we cannot continue safely after an error so we enter HALT state
                        new_message.stream->printf("Entering Alarm/Halt state\n");
                        THEKERNEL->call_event(ON_HALT, nullptr);

                    }else if(!sent_ok) {

                        if(gcode->add_nl)
                            new_message.stream->printf("\r\n");

                        if(!gcode->txt_after_ok.empty()) {
                            new_message.stream->printf("ok %s\r\n", gcode->txt_after_ok.c_str());
                            gcode->txt_after_ok.clear();

                        } else {
                            if(THEKERNEL->is_ok_per_line() || THEKERNEL->is_grbl_mode()) {
                                // only send ok once per line if this is a multi g code line send ok on the last one
                                if(possible_command.empty())
                                    new_message.stream->printf("ok\r\n");
                            } else {
                                // maybe should do the above for all hosts?
                                new_message.stream->printf("ok\r\n");
                            }
                        }
                    }

                    delete gcode;

                } else {
                    // we are uploading and it is the upload stream so so save it
                    if(single_command.substr(0, 3) == "M29") {
                        // done uploading, close file
                        fclose(upload_fd);
                        upload_fd = NULL;
                        uploading = false;
                        upload_filename.clear();
                        upload_stream= nullptr;
                        new_message.stream->printf("Done saving file.\r\nok\r\n");
                        continue;
                    }

                    if(upload_fd == NULL) {
                        // error detected writing to file so discard everything until it stops
                        new_message.stream->printf("ok\r\n");
                        continue;
                    }

                    single_command.append("\n");
                    if(fwrite(single_command.c_str(), 1, single_command.size(), upload_fd) != single_command.size()) {
                        // error writing to file
                        new_message.stream->printf("Error:error writing to file.\r\n");
                        fclose(upload_fd);
                        upload_fd = NULL;
                        continue;

                    } else {
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
        // handle pycam syntax, use last modal group 1 command and resubmit if an X Y Z or F is found on its own line
        char buf[6];
        snprintf(buf, sizeof(buf), "G%d ", modal_group_1);
        possible_command.insert(0, buf);
        goto try_again;

    } else if ( first_char == ';' || first_char == '(' || first_char == ' ' || first_char == '\n' || first_char == '\r' ) {
        // Ignore comments and blank lines
        new_message.stream->printf("ok\n");

    } else {
        // an uppercase non command word on its own (except XYZF) just returns ok, we could add an error but no hosts expect that.
        new_message.stream->printf("ok - ignored\n");
    }
}

