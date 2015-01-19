/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GCODE_DISPATCH_H
#define GCODE_DISPATCH_H

#include "libs/Module.h"

#include <stdio.h>
#include <string>
using std::string;

class GcodeDispatch : public Module
{
public:
    GcodeDispatch();

    virtual void on_module_loaded();
    virtual void on_console_line_received(void *line);
    void on_halt(void *arg);

private:
    int currentline;
    string upload_filename;
    FILE *upload_fd;
    uint8_t last_g;
    struct {
        bool uploading: 1;
        bool halted: 1;
        bool return_error_on_unhandled_gcode:1;
    };
};

#endif
