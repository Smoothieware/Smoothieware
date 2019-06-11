/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"

#include <stdio.h>
#include <string>
#include <map>
#include <vector>
using std::string;

class StreamOutput;

class Player : public Module {
    public:
        Player();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(void *argument);
        void on_halt(void *argument);
        void on_suspend(void *argument);

    private:
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream );
        void suspend_command_brm( string parameters, StreamOutput* stream );
        void resume_command( string parameters, StreamOutput* stream );
        string extract_options(string& args);
        void suspend_part2();

        string filename;
        string after_suspend_gcode;
        string before_resume_gcode;
        string on_boot_gcode;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;

        FILE* current_file_handler;
        long file_size;
        unsigned long played_cnt;
        unsigned long elapsed_secs;
        float saved_position[3]; // only saves XYZ
        std::map<uint16_t, float> saved_temperatures;
        struct {
            bool on_boot_gcode_enable:1;
            bool booted:1;
            bool playing_file:1;
            bool suspended:1;
            bool was_playing_file:1;
            bool leave_heaters_on:1;
            bool override_leave_heaters_on:1;
            uint8_t suspend_loops:4;
        };
};
