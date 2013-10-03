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

#define play_command_checksum           CHECKSUM("play")
#define progress_command_checksum       CHECKSUM("progress")
#define abort_command_checksum          CHECKSUM("abort")
#define cd_command_checksum             CHECKSUM("cd")
#define on_boot_gcode_checksum          CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum   CHECKSUM("on_boot_gcode_enable")

class Player : public Module {
    public:
        Player(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(void *argument);
        string absolute_from_relative( string path );
        void cd_command(   string parameters, StreamOutput* stream );
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );

    private:
        string current_path;
        string filename;

        bool on_boot_gcode_enable;
        bool booted;
        string on_boot_gcode;
        bool playing_file;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;
        FILE* current_file_handler;
        unsigned long file_size, played_cnt;
        unsigned long elapsed_secs;
};

#endif // PLAYER_H
