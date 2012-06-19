/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/


#include "libs/Kernel.h"
#include "Configurator.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"


void Configurator::on_module_loaded(){
//    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
//    this->register_for_event(ON_MAIN_LOOP);
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Configurator::on_console_line_received( void* argument ){
    SerialMessage new_message = *static_cast<SerialMessage*>(argument);
    string possible_command = new_message.message;

    // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
    unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient

    // Act depending on command
    switch( check_sum ){
        case config_get_command_checksum: this->config_get_command(  get_arguments(possible_command), new_message.stream ); break;
        case config_set_command_checksum: this->config_set_command(  get_arguments(possible_command), new_message.stream ); break;
        case config_load_command_checksum: this->config_load_command(  get_arguments(possible_command), new_message.stream ); break;
    }
}

// Process and respond to eeprom gcodes (M50x)
void Configurator::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    if( gcode->has_letter('M') ){
        int code = gcode->get_value('M');
        switch( code ){
            case 500:   this->config_store( gcode, gcode->stream ); break;
            case 501:   this->config_read( gcode, gcode->stream ); break;
            case 502:   this->config_defaults( gcode, gcode->stream ); break;
            case 503:   this->config_print( gcode, gcode->stream ); break;
        }
    }
}

void Configurator::on_main_loop(void* argument){
}

void Configurator::config_get_command( string parameters, StreamOutput* stream ){}
void Configurator::config_set_command( string parameters, StreamOutput* stream ){}
void Configurator::config_load_command( string parameters, StreamOutput* stream ){}

// Write live settings to internal storage
void Configurator::config_store( Gcode* gcode, StreamOutput* stream ){
}

// Read settings from internal storage
void Configurator::config_read( Gcode* gcode, StreamOutput* stream ){
}

// Reset live settings to defaults
void Configurator::config_defaults( Gcode* gcode, StreamOutput* stream ){
}

// Output the current live settings to the stream
void Configurator::config_print( Gcode* gcode, StreamOutput* stream ){
}
