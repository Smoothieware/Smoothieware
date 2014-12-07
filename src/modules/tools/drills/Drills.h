/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DRILLS_MODULE_H
#define DRILLS_MODULE_H

#include "libs/Module.h"

class Gcode;

class Drills : public Module
{
    public:
        Drills();
        virtual ~Drills() {};
        void on_module_loaded();

    private:
        void on_gcode_received(void *argument);
        void update_sticky(Gcode *gcode);
        int send_line(const char* format, ...);
        void simple_cycle(Gcode *gcode);

        bool cycle_started;
        int retract_type;

        float initial_z;
        float r_plane;

        float sticky_z;
        float sticky_r;
        float sticky_f;
};

#endif