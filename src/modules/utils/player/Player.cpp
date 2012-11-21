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


void Player::on_module_loaded(){
    this->playing_file = false;
    this->on_booted = false;
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);

    this->on_boot_file_name = this->kernel->config->value(on_boot_gcode_checksum)->by_default("on_boot.gcode")->as_string();
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    switch( check_sum ){
        case play_command_checksum    : this->play_command(get_arguments(possible_command), new_message.stream ); break;
    }
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput* stream ){

    // Get filename
    string filename          = this->kernel->simpleshell->absolute_from_relative(shift_parameter( parameters ));
    stream->printf("Playing %s\r\n", filename.c_str());
    string options           = shift_parameter( parameters );

    this->current_file_handler = fopen( filename.c_str(), "r");
    if(this->current_file_handler == NULL)
    {
        stream->printf("File not found: %s\r\n", filename.c_str());
        return;
    }
    this->playing_file = true;
    if( options.find_first_of("Qq") == string::npos ){
        this->current_stream = stream;
    }else{
        this->current_stream = new StreamOutput();
    }
}

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
                this->kernel->conveyor->wait_for_queue(2);
                this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
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
