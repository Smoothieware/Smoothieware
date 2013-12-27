/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include <string>
using std::string;
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "utils.h"

#include <stdlib.h>

// This is a gcode object. It reprensents a GCode string/command, an caches some important values about that command for the sake of performance.
// It gets passed around in events, and attached to the queue ( that'll change )
Gcode::Gcode(const string& command, StreamOutput* stream) : command(command), m(0), g(0), add_nl(false), stream(stream) {
    prepare_cached_values();
    this->millimeters_of_travel = 0L;
    this->accepted_by_module=false;
}

Gcode::Gcode(const Gcode& to_copy){
    this->command.assign( to_copy.command );
    this->millimeters_of_travel = to_copy.millimeters_of_travel;
    this->has_m                 = to_copy.has_m;
    this->has_g                 = to_copy.has_g;
    this->m                     = to_copy.m;
    this->g                     = to_copy.g;
    this->add_nl                = to_copy.add_nl;
    this->stream                = to_copy.stream;
    this->accepted_by_module=false;
    this->txt_after_ok.assign( to_copy.txt_after_ok );
}

Gcode& Gcode::operator= (const Gcode& to_copy){
    if( this != &to_copy ){
        this->command.assign( to_copy.command );
        this->millimeters_of_travel = to_copy.millimeters_of_travel;
        this->has_m                 = to_copy.has_m;
        this->has_g                 = to_copy.has_g;
        this->m                     = to_copy.m;
        this->g                     = to_copy.g;
        this->add_nl                = to_copy.add_nl;
        this->stream                = to_copy.stream;
        this->txt_after_ok.assign( to_copy.txt_after_ok );
    }
    this->accepted_by_module=false;
    return *this;
}


// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ){
    //return ( this->command->find( letter ) != string::npos );
    for (std::string::const_iterator c = this->command.cbegin(); c != this->command.cend(); c++) {
        if( *c == letter ){
            return true;
        }
    }
    return false;
}

// Retrieve the value for a given letter
// We don't use the high-level methods of std::string because they call malloc and it's very bad to do that inside of interrupts
float Gcode::get_value( char letter ){
    //__disable_irq();
    const char* cs = command.c_str();
    char* cn = NULL;
    for (; *cs; cs++){
         if( letter == *cs ){
             cs++;
             float r = strtof(cs, &cn);
             if (cn > cs)
                 return r;
         }
    }
    //__enable_irq();
    return 0;
}

int Gcode::get_int( char letter )
{
    const char* cs = command.c_str();
    char* cn = NULL;
    for (; *cs; cs++){
        if( letter == *cs ){
            cs++;
            int r = strtol(cs, &cn, 10);
            if (cn > cs)
                return r;
        }
    }
    return 0;
}

int Gcode::get_num_args(){
    int count = 0;
    for(size_t i=1; i<this->command.length(); i++){
        if( this->command.at(i) >= 'A' && this->command.at(i) <= 'Z' ){
            count++;
        }
    }
    return count;
}

// Cache some of this command's properties, so we don't have to parse the string every time we want to look at them
void Gcode::prepare_cached_values(){
    if( this->has_letter('G') ){
        this->has_g = true;
        this->g = this->get_int('G');
    }else{
        this->has_g = false;
    }
    if( this->has_letter('M') ){
        this->has_m = true;
        this->m = this->get_int('M');
    }else{
        this->has_m = false;
    }
}

void Gcode::mark_as_taken(){
    this->accepted_by_module = true;
}
