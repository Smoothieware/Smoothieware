/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "libs/Kernel.h"
#include "Player.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "PublicDataRequest.h"
#include "PlayerPublicAccess.h"

void Player::on_module_loaded(){
    this->playing_file = false;
    this->booted = false;
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(true)->as_bool();
    this->elapsed_secs= 0;
    this->reply_stream= NULL;
}

void Player::on_second_tick(void*) {
     if (!THEKERNEL->pauser->paused()) this->elapsed_secs++;
}

void Player::on_gcode_received(void *argument) {
    Gcode *gcode = static_cast<Gcode*>(argument);
    string args= get_arguments(gcode->command);
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            gcode->mark_as_taken();
            gcode->stream->printf("SD card ok\r\n");

        }else if (gcode->m == 23) { // select file
            gcode->mark_as_taken();
            // Get filename
            this->filename= "/sd/" + this->absolute_from_relative(shift_parameter( args ));
            this->current_stream = &(StreamOutput::NullStream);

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }
            this->current_file_handler = fopen( this->filename.c_str(), "r");
            // get size of file
            int result = fseek(this->current_file_handler, 0, SEEK_END);
            if (0 != result){
                    gcode->stream->printf("WARNING - Could not get file size\r\n");
                    file_size= -1;
            }else{
                    file_size= ftell(this->current_file_handler);
                    fseek(this->current_file_handler, 0, SEEK_SET);
            }

            if(this->current_file_handler == NULL){
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            }else{
                gcode->stream->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(),file_size);
                gcode->stream->printf("File selected\r\n");
            }

            this->played_cnt= 0;
            this->elapsed_secs= 0;

        }else if (gcode->m == 24) { // start print
            gcode->mark_as_taken();
            if (this->current_file_handler != NULL) {
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream= THEKERNEL->streams;
            }

        }else if (gcode->m == 25) { // pause print
            gcode->mark_as_taken();
            this->playing_file = false;

        }else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            gcode->mark_as_taken();
            if(this->current_file_handler != NULL){
                // abort the print
                abort_command("", gcode->stream);

                // reload the last file opened
                this->current_file_handler = fopen( this->filename.c_str(), "r");

                if(this->current_file_handler == NULL){
                    gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
                }else{
                    // get size of file
                    int result = fseek(this->current_file_handler, 0, SEEK_END);
                    if (0 != result){
                            gcode->stream->printf("WARNING - Could not get file size\r\n");
                            file_size= 0;
                    }else{
                            file_size= ftell(this->current_file_handler);
                            fseek(this->current_file_handler, 0, SEEK_SET);
                    }
                }
            }else{
                gcode->stream->printf("No file loaded\r\n");
            }

        }else if (gcode->m == 27) { // report print progress, in format used by Marlin
            gcode->mark_as_taken();
            progress_command("-b", gcode->stream);

        }else if (gcode->m == 32) { // select file and start print
            gcode->mark_as_taken();
            // Get filename
            this->filename= "/sd/" + this->absolute_from_relative(shift_parameter( args ));
            this->current_stream = &(StreamOutput::NullStream);

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL){
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            }else{
                this->playing_file = true;
            }
        }
    }
}


// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);

    // ignore comments
    if(new_message.message[0] == ';') return;

    string possible_command = new_message.message;

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    if (check_sum == play_command_checksum)
        this->play_command(  get_arguments(possible_command), new_message.stream );
    else if (check_sum == progress_command_checksum)
        this->progress_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == abort_command_checksum)
        this->abort_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == cd_command_checksum)
        this->cd_command(  get_arguments(possible_command), new_message.stream );
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput* stream ){

    // Get filename
    this->filename          = this->absolute_from_relative(shift_parameter( parameters ));
    string options          = shift_parameter( parameters );

    this->current_file_handler = fopen( this->filename.c_str(), "r");
    if(this->current_file_handler == NULL){
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", this->filename.c_str());

    this->playing_file = true;

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ){
        this->current_stream = &(StreamOutput::NullStream);
    }else{
        // we send to the kernels stream as it cannot go away
        this->current_stream = THEKERNEL->streams;
    }

    // get size of file
    int result = fseek(this->current_file_handler, 0, SEEK_END);
    if (0 != result){
            stream->printf("WARNING - Could not get file size\r\n");
            file_size= 0;
    }else{
            file_size= ftell(this->current_file_handler);
            fseek(this->current_file_handler, 0, SEEK_SET);
            stream->printf("  File size %ld\r\n", file_size);
    }
    this->played_cnt= 0;
    this->elapsed_secs= 0;
}

void Player::progress_command( string parameters, StreamOutput* stream ){

    // get options
    string options           = shift_parameter( parameters );

    if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est= 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec= played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est= (file_size - played_cnt) / bytespersec;
        }

        unsigned int pcnt= (file_size - (file_size - played_cnt)) * 100 / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if ( options.find_first_of("Bb") == string::npos ){
            stream->printf("%u %% complete, elapsed time: %lu s", pcnt, this->elapsed_secs);
            if(est > 0){
                stream->printf(", est time: %lu s",  est);
            }
            stream->printf("\r\n");
        }else{
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    }else{
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput* stream ){
    if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }
    playing_file = false;
    played_cnt= 0;
    file_size= 0;
    this->filename= "";
    this->current_stream= NULL;
    fclose(current_file_handler);
    stream->printf("Aborted playing file\r\n");
}

// Convert a path indication ( absolute or relative ) into a path ( absolute )
string Player::absolute_from_relative( string path ){
    if( path[0] == '/' ){ return path; }
    if( path[0] == '.' ){ return this->current_path; }
    return this->current_path + path;
}

// Change current absolute path to provided path
void Player::cd_command( string parameters, StreamOutput* stream ){
    string folder = this->absolute_from_relative( parameters );
    if( folder[folder.length()-1] != '/' ){ folder += "/"; }
    DIR *d;
    d = opendir(folder.c_str());
    if(d == NULL) {
//        stream->printf("Could not open directory %s \r\n", folder.c_str() );
    }else{
        this->current_path = folder;
        closedir(d);
    }
}

void Player::on_main_loop(void* argument){
    if( !this->booted ) {
        this->booted = true;
        if( this->on_boot_gcode_enable ){
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        }else{
            //THEKERNEL->serial->printf("On boot gcode disabled! skipping...\n");
        }
    }

    if( this->playing_file ){
        string buffer;
        bool discard= false;
        int c;
        buffer.reserve(20);
        // Print each line of the file
        while ((c = fgetc(this->current_file_handler)) != EOF){
            if (c == '\n'){
                if(discard) {
                    // we hit a long line and discarded it
                    discard= false;
                    buffer.clear();
                    this->current_stream->printf("Warning: Discarded long line\n");
                    return;
                }
                this->current_stream->printf("%s\n", buffer.c_str());
                struct SerialMessage message;
                message.message = buffer;
                message.stream = &(StreamOutput::NullStream); // we don't really need to see the ok
                // wait for the queue to have enough room that a serial message could still be received before sending
                THEKERNEL->conveyor->wait_for_queue(2);
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                played_cnt += buffer.size();
                buffer.clear();
                return;

            }else if(buffer.size() > 128) {
                // discard rest of line
                discard= true;

            }else{
                buffer += c;
            }
        }

        this->playing_file = false;
        this->filename= "";
        played_cnt= 0;
        file_size= 0;
        fclose(this->current_file_handler);
        this->current_stream= NULL;

        if(this->reply_stream != NULL) {
            // if we were printing from an M command from pronterface we need to send this back
            this->reply_stream->printf("Done printing file\r\n");
            this->reply_stream= NULL;
        }
    }
}

void Player::on_get_public_data(void* argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum)) {
        static bool bool_data;
        bool_data= this->playing_file;
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    }else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(file_size > 0 && playing_file) {
            p.elapsed_secs= this->elapsed_secs;
            p.percent_complete= (this->file_size - (this->file_size - this->played_cnt)) * 100 / this->file_size;
            p.filename= this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    }
}

void Player::on_set_public_data(void* argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    }
}





/*
void Player::on_main_loop(void* argument){
    if( !this->on_booted ){
        this->play_command(this->on_boot_file_name, new StreamOutput());
        this->on_booted = true;
    }
    if( this->playing_file ){
        string buffer;
        int c;
        // Print each line of the file
        while ((c = fgetc(this->current_file_handler)) != EOF){
            if (c == '\n'){
                this->current_stream->printf("%s\n", buffer.c_str());
                struct SerialMessage message;
                message.message = buffer;
                message.stream = this->current_stream;
                // wait for the queue to have enough room that a serial message could still be received before sending
                THEKERNEL->conveyor->wait_for_queue(2);
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                buffer.clear();
                return;
            }else{
                buffer += c;
            }
        };

        fclose(this->current_file_handler);
        this->playing_file = false;
    }
}
*/

