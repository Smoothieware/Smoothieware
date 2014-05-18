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
#include <stdlib.h>

class StreamOutput;

// Object to represent a Gcode command
class Gcode {
    public:
        Gcode(const string&, StreamOutput*);
        Gcode(const Gcode& to_copy);
        Gcode& operator= (const Gcode& to_copy);

        bool   has_letter ( char letter );

        float get_value  ( char letter );

        float get_double ( char letter );
        int    get_int    ( char letter );

        int    get_num_args();
        void   prepare_cached_values();
        void   mark_as_taken();

        string command;
        float millimeters_of_travel;

        bool has_m;
        bool has_g;
        unsigned int m;
        unsigned int g;

        bool add_nl;
        StreamOutput* stream;

        string txt_after_ok;
        bool accepted_by_module;

};
#endif
