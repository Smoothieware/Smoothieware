/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef simpleshell_h
#define simpleshell_h

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Channel.h"

class SimpleShell : public Module
{
public:
    SimpleShell() {}

    void on_module_loaded();
    void on_console_line_received( void *argument );
    void on_gcode_received(void *argument);
    void on_second_tick(void *);

private:
    string absolute_from_relative( string path );
    void ls_command(string parameters, Channel *stream );
    void cd_command(string parameters, Channel *stream );
    void delete_file_command(string parameters, Channel *stream );
    void pwd_command(string parameters, Channel *stream );
    void cat_command(string parameters, Channel *stream );
    void rm_command(string parameters, Channel *stream );
    void break_command(string parameters, Channel *stream );
    void reset_command(string parameters, Channel *stream );
    void dfu_command(string parameters, Channel *stream );
    void help_command(string parameters, Channel *stream );
    void version_command(string parameters, Channel *stream );
    void get_command(string parameters, Channel *stream );
    void set_temp_command(string parameters, Channel *stream );
    void mem_command(string parameters, Channel *stream );

    void net_command( string parameters, Channel *stream);

    bool parse_command(unsigned short cs, string args, Channel *stream);

    typedef void (SimpleShell::*PFUNC)(string parameters, Channel *stream);
    typedef struct {
        unsigned short command_cs;
        PFUNC pfunc;
    } ptentry_t;

    static ptentry_t commands_table[];
    string current_path;
    int reset_delay_secs;
};


#endif
