/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IREMITTER_MODULE_H
#define IREMITTER_MODULE_H

#include "Pin.h"
#include "mbed.h"

class Gcode;

class Iremitter : public Module
{
    public:
        Iremitter();
        virtual ~Iremitter() {};
        void on_module_loaded();
        enum CAMERA {
            CANON,
            CANONWLDC100,
            MINOLTA,
            NIKON,
            OLYMPUS,
            PENTAX,
            SONY,
        };

    private:
        void on_config_reload(void *argument);
        void on_gcode_received(void *argument);
        void on_gcode_execute(void *argument);

        bool is_executable(Gcode *gcode);
        void high_us(unsigned int time, int freq);
        
        void trigger();

        void canon_trigger();
        void canonwldc100_trigger();
        void minolta_trigger();
        void nikon_trigger();
        void olympus_trigger();
        void pentax_trigger();
        void sony_trigger();

        CAMERA       camera;
        unsigned int mcode;
        Pin          pin;
};

#endif

