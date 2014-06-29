/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Pauser.h"
#include "Config.h"
#include "ConfigValue.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "PublicDataRequest.h"
#include "PlayerPublicAccess.h"

#define on_boot_gcode_checksum          CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum   CHECKSUM("on_boot_gcode_enable")

void Player::on_module_loaded()
{
    this->playing_file = false;
    this->current_file_handler = NULL;
    this->booted = false;
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(true)->as_bool();
    this->elapsed_secs = 0;
    this->reply_stream = NULL;
}

void Player::on_second_tick(void *)
{
    if (!THEKERNEL->pauser->paused()) this->elapsed_secs++;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            gcode->mark_as_taken();
            gcode->stream->printf("SD card ok\r\n");

        } else if (gcode->m == 23) { // select file
            gcode->mark_as_taken();
            // Get filename
            this->filename = "/sd/" + shift_parameter( args );
            this->current_stream = &(StreamOutput::NullStream);

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
            gcode->mark_as_taken();
            if (this->current_file_handler != NULL) {
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream = THEKERNEL->streams;
            }

        } else if (gcode->m == 25) { // pause print
            gcode->mark_as_taken();
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            gcode->mark_as_taken();
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
                        this->current_stream = &(StreamOutput::NullStream);
                    }
                }

            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            gcode->mark_as_taken();
            progress_command("-b", gcode->stream);

        } else if (gcode->m == 32) { // select file and start print
            gcode->mark_as_taken();
            // Get filename
            this->filename = "/sd/" + shift_parameter( args );
            this->current_stream = &(StreamOutput::NullStream);

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            } else {
                this->playing_file = true;
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    // ignore comments and blank lines and if this is a G code then also ignore it
    char first_char = new_message.message[0];
    if(strchr(";( \n\rGMTN", first_char) != NULL) return;

    string possible_command = new_message.message;
    string cmd = shift_parameter(possible_command);

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort")
        this->abort_command( possible_command, new_message.stream );
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{

    // Get filename
    this->filename          = absolute_from_relative(shift_parameter( parameters ));
    string options          = shift_parameter( parameters );

    if(this->playing_file) {
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
        this->current_stream = &(StreamOutput::NullStream);
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

        unsigned int pcnt = (file_size - (file_size - played_cnt)) * 100 / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("%u %% complete, elapsed time: %lu s", pcnt, this->elapsed_secs);
            if(est > 0) {
                stream->printf(", est time: %lu s",  est);
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
    playing_file = false;
    played_cnt = 0;
    file_size = 0;
    this->filename = "";
    this->current_stream = NULL;
    fclose(current_file_handler);
    current_file_handler = NULL;
    stream->printf("Aborted playing or paused file\r\n");
}

void Player::on_main_loop(void *argument)
{
    if( !this->booted ) {
        this->booted = true;
        if( this->on_boot_gcode_enable ) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        } else {
            //THEKERNEL->serial->printf("On boot gcode disabled! skipping...\n");
        }
    }

    if( this->playing_file ) {
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

                this->current_stream->printf("%s", buf);
                struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream;

                // waits for the queue to have enough room
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                played_cnt += len;
                return; // we feed one line per main loop

            } else {
                // discard long line
                this->current_stream->printf("Warning: Discarded long line\n");
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

    if(pdr->second_element_is(is_playing_checksum)) {
        static bool bool_data;
        bool_data = this->playing_file;
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
