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
#include "libs/StreamOutput.h"


#define ls_command_checksum       CHECKSUM("ls")
#define cd_command_checksum       CHECKSUM("cd")
#define pwd_command_checksum      CHECKSUM("pwd")
#define cat_command_checksum      CHECKSUM("cat")
#define play_command_checksum     CHECKSUM("play")
#define progress_command_checksum CHECKSUM("progress")
#define abort_command_checksum    CHECKSUM("abort")
#define reset_command_checksum    CHECKSUM("reset")
#define dfu_command_checksum      CHECKSUM("dfu")
#define break_command_checksum    CHECKSUM("break")
#define help_command_checksum     CHECKSUM("help")

class SimpleShell : public Module {
    public:
        SimpleShell(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        string absolute_from_relative( string path );
        void ls_command(   string parameters, StreamOutput* stream );
        void cd_command(   string parameters, StreamOutput* stream );
        void pwd_command(  string parameters, StreamOutput* stream );
        void cat_command(  string parameters, StreamOutput* stream );
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void break_command(string parameters, StreamOutput* stream );
		void reset_command(string parameters, StreamOutput* stream );
		void help_command(string parameters, StreamOutput* stream );
		
	private:
        string current_path;
		bool playing_file;
		
        StreamOutput* current_stream;
		FILE* current_file_handler;
		long file_size, played_cnt;
};


#endif
