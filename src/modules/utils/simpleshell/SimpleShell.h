/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"

#include <functional>
#include <string>
using std::string;

class StreamOutput;

class SimpleShell : public Module
{
public:
    SimpleShell() {}

    void on_module_loaded();
    void on_console_line_received( void *argument );
    void on_gcode_received(void *argument);
    void on_second_tick(void *);
    static bool parse_command(const char *cmd, string args, StreamOutput *stream);
    static void print_mem(StreamOutput *stream) { mem_command("", stream); }
    static void version_command(string parameters, StreamOutput *stream );

private:
    static void ls_command(string parameters, StreamOutput *stream );
    static void cd_command(string parameters, StreamOutput *stream );
    static void delete_file_command(string parameters, StreamOutput *stream );
    static void pwd_command(string parameters, StreamOutput *stream );
    static void cat_command(string parameters, StreamOutput *stream );
    static void rm_command(string parameters, StreamOutput *stream );
    static void mv_command(string parameters, StreamOutput *stream );
    static void mkdir_command(string parameters, StreamOutput *stream );
    static void upload_command(string parameters, StreamOutput *stream );
    static void break_command(string parameters, StreamOutput *stream );
    static void reset_command(string parameters, StreamOutput *stream );
    static void dfu_command(string parameters, StreamOutput *stream );
    static void help_command(string parameters, StreamOutput *stream );
    static void get_command(string parameters, StreamOutput *stream );
    static void set_temp_command(string parameters, StreamOutput *stream );
    static void calc_thermistor_command( string parameters, StreamOutput *stream);
    static void print_thermistors_command( string parameters, StreamOutput *stream);
    static void md5sum_command( string parameters, StreamOutput *stream);
    static void grblDP_command( string parameters, StreamOutput *stream);

    static void switch_command(string parameters, StreamOutput *stream );
    static void mem_command(string parameters, StreamOutput *stream );

    static void net_command( string parameters, StreamOutput *stream);

    static void load_command( string parameters, StreamOutput *stream);
    static void save_command( string parameters, StreamOutput *stream);

    static void remount_command( string parameters, StreamOutput *stream);

    static void test_command( string parameters, StreamOutput *stream);

    typedef void (*PFUNC)(string parameters, StreamOutput *stream);
    typedef struct {
        const char *command;
        const PFUNC func;
    } const ptentry_t;

    static const ptentry_t commands_table[];
    static int reset_delay_secs;
};
