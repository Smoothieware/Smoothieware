/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DRILLINGCYCLES_MODULE_H
#define DRILLINGCYCLES_MODULE_H

#include "libs/Module.h"

class Gcode;

class Drillingcycles : public Module
{
    public:
        Drillingcycles();
        virtual ~Drillingcycles() {};
        void on_module_loaded();

    private:
        void on_config_reload(void *argument);
        void on_gcode_received(void *argument);
        void reset_sticky();
        void update_sticky(Gcode *gcode);
        int  send_gcode(const char* format, ...);
        void make_hole(Gcode *gcode);
        void peck_hole();

        bool cycle_started; // cycle status
        int  retract_type;  // rretract type

        float initial_z;    // Initial-Z
        float r_plane;      // R-Plane

        float sticky_z;     // final depth
        float sticky_r;     // R-Plane
        float sticky_f;     // feedrate

        float sticky_q;     // depth increment
        int   sticky_p;     // dwell pause

        int   dwell_units;  // units for dwell
};

#endif
