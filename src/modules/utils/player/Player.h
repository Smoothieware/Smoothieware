/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef PLAYER_H
#define PLAYER_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/StreamOutput.h"


#define play_command_checksum    17335
#define on_boot_gcode_checksum   42838

class Player : public Module {
    public:
        Player(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void play_command( string parameters, StreamOutput* stream );

        bool on_booted;
        string on_boot_file_name;
        bool playing_file;
        StreamOutput* current_stream;
        FILE* current_file_handler;
};

#endif // PLAYER_H
