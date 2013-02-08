/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "libs/Kernel.h"
#include "SimpleShell.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "modules/robot/Player.h"


void SimpleShell::on_module_loaded(){
    this->current_path = "/";
    this->playing_file = false;
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
}

// When a new line is received, check if it is a command, and if it is, act upon it
void SimpleShell::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    //new_message.stream->printf("Received %s\r\n", possible_command.c_str());

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    if (check_sum == ls_command_checksum)
        this->ls_command(  get_arguments(possible_command), new_message.stream );
    else if (check_sum == cd_command_checksum)
        this->cd_command(  get_arguments(possible_command), new_message.stream );
    else if (check_sum == pwd_command_checksum)
        this->pwd_command( get_arguments(possible_command), new_message.stream );
    else if (check_sum == cat_command_checksum)
        this->cat_command( get_arguments(possible_command), new_message.stream );
    else if (check_sum == play_command_checksum)
        this->play_command(get_arguments(possible_command), new_message.stream );
    else if (check_sum == reset_command_checksum)
        this->reset_command(get_arguments(possible_command),new_message.stream );
    else if (check_sum == dfu_command_checksum)
        this->reset_command(get_arguments(possible_command),new_message.stream );
}

// Convert a path indication ( absolute or relative ) into a path ( absolute )
string SimpleShell::absolute_from_relative( string path ){
    if( path[0] == '/' ){ return path; }
    if( path[0] == '.' ){ return this->current_path; }
    return this->current_path + path;
}

// Act upon an ls command
// Convert the first parameter into an absolute path, then list the files in that path
void SimpleShell::ls_command( string parameters, StreamOutput* stream ){
    string folder = this->absolute_from_relative( parameters );
    DIR* d;
    struct dirent* p;
    d = opendir(folder.c_str());
    if(d != NULL) {
		while((p = readdir(d)) != NULL) { stream->printf("%s\r\n", lc(string(p->d_name)).c_str()); }
		closedir(d);
    } else {
        stream->printf("Could not open directory %s \r\n", folder.c_str());
    }
}

// Change current absolute path to provided path
void SimpleShell::cd_command( string parameters, StreamOutput* stream ){
    string folder = this->absolute_from_relative( parameters );
    if( folder[folder.length()-1] != '/' ){ folder += "/"; }
    DIR *d;
    d = opendir(folder.c_str());
    if(d == NULL) {
        stream->printf("Could not open directory %s \r\n", folder.c_str() );
    }else{
		this->current_path = folder;
		closedir(d);
    }
}

// Responds with the present working directory
void SimpleShell::pwd_command( string parameters, StreamOutput* stream ){
    stream->printf("%s\r\n", this->current_path.c_str());
}

// Output the contents of a file, first parameter is the filename, second is the limit ( in number of lines to output )
void SimpleShell::cat_command( string parameters, StreamOutput* stream ){

    // Get parameters ( filename and line limit )
    string filename          = this->absolute_from_relative(shift_parameter( parameters ));
    string limit_paramater   = shift_parameter( parameters );
    int limit = -1;
    if( limit_paramater != "" ){ limit = int(atof(limit_paramater.c_str())); }

    // Open file
    FILE *lp = fopen(filename.c_str(), "r");
    if(lp == NULL) {
        stream->printf("File not found: %s\r\n", filename.c_str());
        return;
    }
    string buffer;
    int c;
    int newlines = 0;

    // Print each line of the file
    while ((c = fgetc (lp)) != EOF){
        buffer.append((char *)&c, 1);
        if( char(c) == '\n' ){
            newlines++;
            stream->printf("%s", buffer.c_str());
            buffer.clear();
        }
        if( newlines == limit ){ break; }
    };
    fclose(lp);

}

// Play a gcode file by considering each line as if it was received on the serial console
void SimpleShell::play_command( string parameters, StreamOutput* stream ){

    // Get filename
    string filename          = this->absolute_from_relative(shift_parameter( parameters ));
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
        this->current_stream = kernel->streams;
    }
}

// Reset the system
void SimpleShell::reset_command( string parameters, StreamOutput* stream){
    stream->printf("Smoothie out. Peace.\r\n");
    system_reset();
}

void SimpleShell::on_main_loop(void* argument){

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
                this->kernel->player->wait_for_queue(2);
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
