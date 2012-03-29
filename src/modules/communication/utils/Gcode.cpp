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

#include <stdlib.h>


Gcode::Gcode(){}

// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ){
    //return ( this->command->find( letter ) != string::npos );
    for (size_t i=0; i < this->command.length(); i++){
        if( this->command.at(i) == letter ){
            return true;
        }
    }
    return false;
}

// Retrieve the value for a given letter
// We don't use the high-level methods of std::string because they call malloc and it's very bad to do that inside of interrupts
double Gcode::get_value( char letter ){
    //__disable_irq();
    for (size_t i=0; i <= this->command.length()-1; i++){
         if( letter == this->command.at(i) ){
            size_t beginning = i+1;
            char buffer[20];
            for(size_t j=beginning; j <= this->command.length(); j++){
                char c;
                if( j == this->command.length() ){ c = ';'; }else{ c = this->command.at(j); }
                if( c != '.' && c != '-' && ( c < '0' || c > '9' ) ){
                    buffer[j-beginning] = '\0';
                    //__enable_irq();
                    return atof(buffer); 
                }else{
                    buffer[j-beginning] = c;
                }
            }
         }
    }
    //__enable_irq();
    return 0; 
}
