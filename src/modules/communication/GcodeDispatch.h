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

class StreamOutput;

class GcodeDispatch : public Module
{
public:
    GcodeDispatch();

    virtual void on_module_loaded();
    virtual void on_console_line_received(void *line);

    uint8_t get_modal_command() const { return modal_group_1<4 ? modal_group_1 : 0; }
private:
    int currentline;
    string upload_filename;
    FILE *upload_fd;
    StreamOutput* upload_stream{nullptr};
    uint8_t modal_group_1;
    struct {
        bool uploading: 1;
    };
};

#endif
