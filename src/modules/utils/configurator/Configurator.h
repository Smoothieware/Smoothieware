/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef configurator_h
#define configurator_h

#include "Module.h"

#include <string>
using std::string;

class StreamOutput;

class Configurator : public Module
{
public:
    Configurator() {}

    void on_module_loaded();
    void on_console_line_received( void *argument );
    void on_main_loop( void *argument );

    void config_get_command( string parameters, StreamOutput *stream );
    void config_set_command( string parameters, StreamOutput *stream );
    void config_load_command(string parameters, StreamOutput *stream );
};


#endif // configurator_h
