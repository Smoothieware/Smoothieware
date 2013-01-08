/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include <string>
using std::string;

#include <cstring>

#include "Gcode.h"
#include "libs/StreamOutput.h"

#include <stdlib.h>

#include "utils.h"

Gcode::Gcode(){}

// Whether or not a Gcode has a letter
bool Gcode::has_letter( char letter ){
    //return ( this->command->find( letter ) != string::npos );
    return (find_first_of(command, letter) >= 0);
}

// Retrieve the value for a given letter
// We don't use the high-level methods of std::string because they call malloc and it's very bad to do that inside of interrupts
double Gcode::get_value( char letter ){
    //__disable_irq();
    int j = find_first_of(command, letter);
    if (j >= 0)
    {
        do {
            j++;
        } while ((command[j] == ' ') || (command[j] == '\t'));
        if (((command[j] >= '0') && (command[j] <= '9')) || (command[j] == '-'))
        {
            char* end = command + j;
            double r = strtod(command + j, &end);
            if (end > (command + j))
                return r;
        }
    }
    //__enable_irq();
    return 0.0;
}

// int Gcode::get_value( char letter )
// {
//     int j = find_first_of(command, letter);
//     if (j >= 0)
//     {
//         do {
//             j++;
//         } while ((command[j] == ' ') || (command[j] == '\t'));
//         if (((command[j] >= '0') && (command[j] <= '9')) || (command[j] == '-'))
//             return atoi(command[j]);
//     }
//     return 0;
// }

int Gcode::get_num_args(){
    int count = 0;
    for (int i = strlen(command); i; i--)
    {
        if ((command[i] >= 'A') && (command[i] <= 'Z'))
            count++;
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
