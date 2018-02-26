/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "SDFAT.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "ExtruderPublicAccess.h"

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define on_boot_gcode_checksum            CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum     CHECKSUM("on_boot_gcode_enable")
#define after_suspend_gcode_checksum      CHECKSUM("after_suspend_gcode")
#define before_resume_gcode_checksum      CHECKSUM("before_resume_gcode")
#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")

extern SDFAT mounter;

Player::Player()
{
    this->playing_file = false;
    this->current_file_handler = nullptr;
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->suspended= false;
    this->suspend_loops= 0;
}

void Player::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(true)->as_bool();

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->by_default("")->as_string();
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->by_default("")->as_string();
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();
}

void Player::on_halt(void* argument)
{
    if(argument == nullptr && this->playing_file ) {
        abort_command("1", &(StreamOutput::NullStream));
    }
}

void Player::on_second_tick(void *)
{
    if(this->playing_file) this->elapsed_secs++;
}

// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            mounter.remount();
            gcode->stream->printf("SD card ok\r\n");

        } else if (gcode->m == 23) { // select file
            this->filename = "/sd/" + args; // filename is whatever is in args
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }
            this->current_file_handler = fopen( this->filename.c_str(), "r");

            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
                return;

            } else {
                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                    this->file_size = 0;
                } else {
                    this->file_size = ftell(this->current_file_handler);
                    fseek(this->current_file_handler, 0, SEEK_SET);
                }
                gcode->stream->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);
                gcode->stream->printf("File selected\r\n");
            }


            this->played_cnt = 0;
            this->elapsed_secs = 0;

        } else if (gcode->m == 24) { // start print
            if (this->current_file_handler != NULL) {
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream = THEKERNEL->streams;
            }

        } else if (gcode->m == 25) { // pause print
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            if(this->current_file_handler != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->current_file_handler = fopen(currentfn.c_str() , "r");

                    if(this->current_file_handler == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->filename = currentfn;
                        this->file_size = old_size;
                        this->current_stream = nullptr;
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            progress_command("-b", gcode->stream);

        } else if (gcode->m == 32) { // select file and start print
            // Get filename
            this->filename = "/sd/" + args; // filename is whatever is in args including spaces
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            } else {
                this->playing_file = true;

                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                        file_size = 0;
                } else {
                        file_size = ftell(this->current_file_handler);
                        fseek(this->current_file_handler, 0, SEEK_SET);
                }
            }

            this->played_cnt = 0;
            this->elapsed_secs = 0;

        } else if (gcode->m == 600) { // suspend print, Not entirely Marlin compliant, M600.1 will leave the heaters on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);

        } else if (gcode->m == 601) { // resume print
            this->resume_command("", gcode->stream);
        }

    }else if(gcode->has_g) {
        if(gcode->g == 28) { // homing cancels suspend
            if(this->suspended) {
                // clean up
                this->suspended= false;
                THEROBOT->pop_state();
                this->saved_temperatures.clear();
                this->was_playing_file= false;
                this->suspend_loops= 0;
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort") {
        this->abort_command( possible_command, new_message.stream );
    }else if (cmd == "suspend") {
        this->suspend_command( possible_command, new_message.stream );
    }else if (cmd == "resume") {
        this->resume_command( possible_command, new_message.stream );
    }
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{
    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(parameters);

    if(this->playing_file || this->suspended) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    if(this->current_file_handler != NULL) { // must have been a paused print
        fclose(this->current_file_handler);
    }

    this->current_file_handler = fopen( this->filename.c_str(), "r");
    if(this->current_file_handler == NULL) {
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", this->filename.c_str());

    this->playing_file = true;

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        // we send to the kernels stream as it cannot go away
        this->current_stream = THEKERNEL->streams;
    }

    // get size of file
    int result = fseek(this->current_file_handler, 0, SEEK_END);
    if (0 != result) {
        stream->printf("WARNING - Could not get file size\r\n");
        file_size = 0;
    } else {
        file_size = ftell(this->current_file_handler);
        fseek(this->current_file_handler, 0, SEEK_SET);
        stream->printf("  File size %ld\r\n", file_size);
    }
    this->played_cnt = 0;
    this->elapsed_secs = 0;
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && current_file_handler != NULL) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", played_cnt, file_size);
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est = 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec = played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est = (file_size - played_cnt) / bytespersec;
        }

        float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), this->elapsed_secs / 3600, (this->elapsed_secs % 3600) / 60, this->elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput *stream )
{
    if(!playing_file && current_file_handler == NULL) {
        stream->printf("Not currently playing\r\n");
        return;
    }
    suspended= false;
    playing_file = false;
    played_cnt = 0;
    file_size = 0;
    this->filename = "";
    this->current_stream = NULL;
    fclose(current_file_handler);
    current_file_handler = NULL;
    if(parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THEKERNEL->conveyor->flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT->reset_position_from_current_actuator_position();
        stream->printf("Aborted playing or paused file. Please turn any heaters off manually\r\n");
    }
}

void Player::on_main_loop(void *argument)
{
    if(suspended && suspend_loops > 0) {
        // if we are suspended we need to allow main loop to cycle a few times then finish off the suspend processing
        if(--suspend_loops == 0) {
            suspend_part2();
            return;
        }
    }

    if( !this->booted ) {
        this->booted = true;
        if( this->on_boot_gcode_enable ) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        } else {
            //THEKERNEL->serial->printf("On boot gcode disabled! skipping...\n");
        }
    }

    if( this->playing_file ) {
        if(THEKERNEL->is_halted()) {
            return;
        }

        char buf[130]; // lines upto 128 characters are allowed, anything longer is discarded
        bool discard = false;

        while(fgets(buf, sizeof(buf), this->current_file_handler) != NULL) {
            int len = strlen(buf);
            if(len == 0) continue; // empty line? should not be possible
            if(buf[len - 1] == '\n' || feof(this->current_file_handler)) {
                if(discard) { // we are discarding a long line
                    discard = false;
                    continue;
                }
                if(len == 1) continue; // empty line

                if(this->current_stream != nullptr) {
                    this->current_stream->printf("%s", buf);
                }

                struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;

                // waits for the queue to have enough room
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                played_cnt += len;
                return; // we feed one line per main loop

            } else {
                // discard long line
                if(this->current_stream != nullptr) { this->current_stream->printf("Warning: Discarded long line\n"); }
                discard = true;
            }
        }

        this->playing_file = false;
        this->filename = "";
        played_cnt = 0;
        file_size = 0;
        fclose(this->current_file_handler);
        current_file_handler = NULL;
        this->current_stream = NULL;

        if(this->reply_stream != NULL) {
            // if we were printing from an M command from pronterface we need to send this back
            this->reply_stream->printf("Done printing file\r\n");
            this->reply_stream = NULL;
        }
    }
}

void Player::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum) || pdr->second_element_is(is_suspended_checksum)) {
        static bool bool_data;
        bool_data = pdr->second_element_is(is_playing_checksum) ? this->playing_file : this->suspended;
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(file_size > 0 && playing_file) {
            p.elapsed_secs = this->elapsed_secs;
            p.percent_complete = (this->file_size - (this->file_size - this->played_cnt)) * 100 / this->file_size;
            p.filename = this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    }
}

void Player::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    }
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
1a. loop on_main_loop several times to clear any buffered commmands
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if(suspended) {
        stream->printf("Already suspended\n");
        return;
    }

    stream->printf("Suspending print, waiting for queue to empty...\n");

    // override the leave_heaters_on setting
    this->override_leave_heaters_on= (parameters == "h");

    suspended= true;
    if( this->playing_file ) {
        // pause an sd print
        this->playing_file = false;
        this->was_playing_file= true;
    }else{
        // send pause to upstream host, we send it on all ports as we don't know which it is on
        THEKERNEL->streams->printf("// action:pause\r\n");
        this->was_playing_file= false;
    }

    // we need to allow main loop to cycle a few times to clear any buffered commands in the serial streams etc
    suspend_loops= 10;
}

// this completes the suspend
void Player::suspend_part2()
{
    //  need to use streams here as the original stream may have changed
    THEKERNEL->streams->printf("// Waiting for queue to empty (Host must stop sending)...\n");
    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    THEKERNEL->streams->printf("// Saving current state...\n");

    // save current XYZ position
    THEROBOT->get_axis_position(this->saved_position);

    // save current extruder state
    PublicData::set_value( extruder_checksum, save_state_checksum, nullptr );

    // save state use M120
    THEROBOT->push_state();

    // TODO retract by optional amount...

    this->saved_temperatures.clear();
    if(!this->leave_heaters_on && !this->override_leave_heaters_on) {
        // save current temperatures, get a vector of all the controllers data
        std::vector<struct pad_temperature> controllers;
        bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
        if (ok) {
            // query each heater and save the target temperature if on
            for (auto &c : controllers) {
                // TODO see if in exclude list
                if(c.target_temperature > 0) {
                    this->saved_temperatures[c.id]= c.target_temperature;
                }
            }
        }

        // turn off heaters that were on
        for(auto& h : this->saved_temperatures) {
            float t= 0;
            PublicData::set_value( temperature_control_checksum, h.first, &t );
        }
    }

    // execute optional gcode if defined
    if(!after_suspend_gcode.empty()) {
        struct SerialMessage message;
        message.message = after_suspend_gcode;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    THEKERNEL->streams->printf("// Print Suspended, enter resume to continue printing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if(!suspended) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("resuming print...\n");

    // wait for them to reach temp
    if(!this->saved_temperatures.empty()) {
        // set heaters to saved temps
        for(auto& h : this->saved_temperatures) {
            float t= h.second;
            PublicData::set_value( temperature_control_checksum, h.first, &t );
        }
        stream->printf("Waiting for heaters...\n");
        bool wait= true;
        uint32_t tus= us_ticker_read(); // mbed call
        while(wait) {
            wait= false;

            bool timeup= false;
            if((us_ticker_read() - tus) >= 1000000) { // print every 1 second
                timeup= true;
                tus= us_ticker_read(); // mbed call
            }

            for(auto& h : this->saved_temperatures) {
                struct pad_temperature temp;
                if(PublicData::get_value( temperature_control_checksum, current_temperature_checksum, h.first, &temp )) {
                    if(timeup)
                        stream->printf("%s:%3.1f /%3.1f @%d ", temp.designator.c_str(), temp.current_temperature, ((temp.target_temperature == -1) ? 0.0 : temp.target_temperature), temp.pwm);
                    wait= wait || (temp.current_temperature < h.second);
                }
            }
            if(timeup) stream->printf("\n");

            if(wait)
                THEKERNEL->call_event(ON_IDLE, this);

            if(THEKERNEL->is_halted()) {
                // abort temp wait and rest of resume
                THEKERNEL->streams->printf("Resume aborted by kill\n");
                THEROBOT->pop_state();
                this->saved_temperatures.clear();
                suspended= false;
                return;
            }
        }
    }

    // execute optional gcode if defined
    if(!before_resume_gcode.empty()) {
        stream->printf("Executing before resume gcode...\n");
        struct SerialMessage message;
        message.message = before_resume_gcode;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    // Restore position
    stream->printf("Restoring saved XYZ positions and state...\n");
    THEROBOT->pop_state();
    bool abs_mode= THEROBOT->absolute_mode; // what mode we were in
    // force absolute mode for restoring position, then set to the saved relative/absolute mode
    THEROBOT->absolute_mode= true;
    {
        // NOTE position was saved in MCS so must use G53 to restore position
        char buf[128];
        snprintf(buf, sizeof(buf), "G53 G0 X%f Y%f Z%f", saved_position[0], saved_position[1], saved_position[2]);
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }
    THEROBOT->absolute_mode= abs_mode;

    // restore extruder state
    PublicData::set_value( extruder_checksum, restore_state_checksum, nullptr );

    stream->printf("Resuming print\n");

    if(this->was_playing_file) {
        this->playing_file = true;
        this->was_playing_file= false;
    }else{
        // Send resume to host
        THEKERNEL->streams->printf("// action:resume\r\n");
    }

    // clean up
    this->saved_temperatures.clear();
    suspended= false;
}
