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


Gcode::Gcode(const string& command, StreamOutput* stream) :
  command(command), m(0), g(0), add_nl(false), stream(stream)
{
    prepare_cached_values();
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
double Gcode::get_value( char letter ){
    //__disable_irq();
    const char* cs = command.c_str();
    char* cn = NULL;
    for (; *cs; cs++){
         if( letter == *cs ){
             cs++;
             double r = strtod(cs, &cn);
             if (cn > cs)
                 return r;
         }
    }
    //__enable_irq();
    return 0;
}

int Gcode::get_int( char letter )
{
    const char* buffer = command.c_str();
    for (int i = 0; buffer[i]; i++)
    {
        if( letter == buffer[i] )
        {
            for(int j = i + 1; buffer[j]; j++)
            {
                if( is_numeric(buffer[j]) )
                {
                    const char* endptr = &buffer[j];
                    int r = strtol(&buffer[j], const_cast<char**>(&endptr), 10);
                    if (endptr > command.c_str())
                        return r;
                    return 0;
                }
                else if ( is_whitespace(buffer[j]) == false )
                    return 0;
            }
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

void Gcode::prepare_cached_values(){
    if( this->has_letter('G') ){
        this->has_g = true;
        this->g = this->get_value('G');
    }else{
        this->has_g = false;
    }
    if( this->has_letter('M') ){
        this->has_m = true;
        this->m = this->get_value('M');
    }else{
        this->has_m = false;
    }
}
