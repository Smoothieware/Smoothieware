#ifndef simpleshell_h
#define simpleshell_h

#include "mbed.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"


#define ls_command_checksum      19679 
#define cd_command_checksum      11207
#define cat_command_checksum     24889


class SimpleShell : public Module {
    public: 
        SimpleShell(){}

        void on_module_loaded();
        void on_console_line_received( void* argument );
        string get_arguments( string possible_command );
        string absolute_from_relative( string path );
        void ls_command( string parameters );
        void cd_command( string parameters );
        void cat_command( string parameters );
        
        string current_path;
};


#endif
