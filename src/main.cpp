/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "mbed.h"
#include "libs/Kernel.h"
#include "modules/tools/laser/Laser.h"
#include "modules/tools/extruder/Extruder.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "libs/SDFileSystem.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

SDFileSystem sd(p5, p6, p7, p8, "sd");
//LocalFileSystem local("local");

//void checksum( Kernel* kernel, string key ){ kernel->serial->printf("%s: %u\r\n", key.c_str(), kernel->config->get_checksum(key)); }

#define ls_command_checksum      19679 
#define cd_command_checksum      11207
#define cat_command_checksum     24889

class SimpleShell : public Module {
    public: 
        SimpleShell(){}

        void on_module_loaded(){
            this->current_path = "/";
            this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
        }

        // When a new line is received, check if it is a command, and if it is, act upon it
        void on_console_line_received( void* argument ){
            string possible_command = *static_cast<string*>(argument);

            // We don't compare to a string but to a checksum of that string, this saves some space in flash memory
            unsigned short check_sum = get_checksum( possible_command.substr(0,possible_command.find_first_of(" \r\n")) );  // todo: put this method somewhere more convenient
            
            // Act depending on command
            switch( check_sum ){
                case ls_command_checksum      : this->ls_command( this->get_arguments(possible_command)); break;
                case cd_command_checksum      : this->cd_command( this->get_arguments(possible_command)); break;
                case cat_command_checksum     : this->cat_command(this->get_arguments(possible_command)); break;
            }
        }

        // Separate command from arguments
        string get_arguments( string possible_command ){
            size_t beginning = possible_command.find_first_of(" ");
            if( beginning == string::npos ){ return ""; } 
            return possible_command.substr( beginning+1, possible_command.size() - beginning);
        }

        // Convert a path indication ( absolute or relative ) into a path ( absolute )
        string absolute_from_relative( string path ){
            if( path[0] == '/' ){ return path; }
            if( path[0] == '.' ){ return this->current_path; } 
            return this->current_path + path;
        }

        // Act upon an ls command
        // Convert the first parameter into an absolute path, then list the files in that path
        void ls_command( string parameters ){
            string folder = this->absolute_from_relative( parameters );
            DIR* d;
            struct dirent* p;
            d = opendir(folder.c_str());
            if(d != NULL) {
                while((p = readdir(d)) != NULL) { this->kernel->serial->printf("%s\r\n", lc(string(p->d_name)).c_str()); }
            } else {
                this->kernel->serial->printf("Could not open directory %s \r\n", folder.c_str());
            }
        }

        // Change current absolute path to provided path
        void cd_command( string parameters ){
            string folder = this->absolute_from_relative( parameters );
            if( folder[folder.length()-1] != '/' ){ folder += "/"; }
            DIR *d;
            struct dirent *p;
            d = opendir(folder.c_str());
            if(d == NULL) { 
                this->kernel->serial->printf("Could not open directory %s \r\n", folder.c_str() ); 
            }else{
                this->current_path = folder;
            }
        }

        // Output the contents of a file, first parameter is the filename, second is the limit ( in number of lines to output )
        void cat_command( string parameters ){
            
            // Get parameters ( filename and line limit ) 
            string filename          = this->absolute_from_relative(shift_parameter( parameters ));
            string limit_paramater   = shift_parameter( parameters );
            int limit = -1;
            if( limit_paramater != "" ){ limit = int(atof(limit_paramater.c_str())); }
           
            // Open file 
            FILE *lp = fopen(filename.c_str(), "r");
            string buffer;
            int c;
            int newlines = 0; 
            
            // Print each line of the file
            while ((c = fgetc (lp)) != EOF){
                if( char(c) == '\n' ){  newlines++; }
                this->kernel->serial->putc(c); 
                if( newlines == limit ){ break; }
            }; 
            fclose(lp);
        
        }


        string current_path;


};



int main() {

    Kernel* kernel = new Kernel();
    
    kernel->serial->printf("Smoothie ( grbl port ) version 0.1a \r\nstart\r\n");

    kernel->add_module( new Laser(p21) );
    //kernel->add_module( new Extruder(p22) );
    kernel->add_module( new SimpleShell() );

    while(1){
        kernel->call_event(ON_MAIN_LOOP);
    }

}
