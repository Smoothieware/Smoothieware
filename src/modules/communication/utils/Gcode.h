/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/


#ifndef GCODE_H
#define GCODE_H
#include <string>
using std::string;
#include "libs/StreamOutput.h"
// Object to represent a Gcode comman
#include <stdlib.h>

class Gcode {
    public:
        Gcode();
        bool has_letter( char letter );
        double get_value ( char letter );

        string command;
        double millimeters_of_travel;
        bool call_on_gcode_execute_event_immediatly;
        bool on_gcode_execute_event_called;

        StreamOutput* stream;
};
#endif
