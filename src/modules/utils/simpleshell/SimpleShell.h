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


#define ls_command_checksum      19679 
#define cd_command_checksum      11207
#define cat_command_checksum     24889
#define play_command_checksum    17335

class SimpleShell : public Module {
    public: 
        SimpleShell(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument ); 
        string absolute_from_relative( string path );
        void ls_command(   string parameters, Stream* stream );
        void cd_command(   string parameters, Stream* stream );
        void cat_command(  string parameters, Stream* stream );
        void play_command( string parameters, Stream* stream );       

        string current_path;
        bool playing_file;
        Stream* current_stream;
        FILE* current_file_handler;
};


#endif
