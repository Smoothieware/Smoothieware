/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/


#ifndef configurator_h
#define configurator_h

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/StreamOutput.h"


#define CONF_NONE       0
#define CONF_ROM        1
#define CONF_SD         2
#define CONF_EEPROM     3

#define config_get_command_checksum        46310    // "config-get"
#define config_set_command_checksum        55538    // "config-set"
#define config_load_command_checksum       3143     // "config-load"

class Configurator : public Module {
    public:
        Configurator(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_gcode_execute( void* argument );
        void on_main_loop( void* argument );

        void config_get_command( string parameters, StreamOutput* stream ); 
        void config_set_command( string parameters, StreamOutput* stream ); 
        void config_load_command(string parameters, StreamOutput* stream );
};


#endif // configurator_h
