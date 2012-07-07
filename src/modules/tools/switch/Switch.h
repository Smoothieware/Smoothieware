/*  
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef SWITCH_H
#define SWITCH_H

#include "libs/Pin.h"
#include <math.h>

#define    switch_checksum            15508 
#define    on_m_code_checksum         29094 
#define    off_m_code_checksum        14853 
#define    output_pin_checksum        18779 
#define    startup_state_checksum     37528 

class Switch : public Module {
    public:
        Switch();
        Switch(uint16_t name);
        
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_execute(void* argument);

        uint16_t name_checksum;
        uint16_t on_m_code;
        uint16_t off_m_code;
        Pin*     output_pin;
};

#endif
