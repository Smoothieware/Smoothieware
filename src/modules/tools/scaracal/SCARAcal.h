/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SCARACAL_H_
#define SCARACAL_H_

#include "Module.h"
#include "Pin.h"

class StepperMotor;
class Gcode;
class StreamOutput;

class SCARAcal: public Module
{

public:
    void on_module_loaded();
    void on_config_reload(void *argument);

    void on_gcode_received(void *argument);
//    void on_idle(void *argument);


private:
    void home();
    bool set_trim(float x, float y, float z, StreamOutput *stream);
    bool get_trim(float& x, float& y, float& z);

    bool set_home_offset(float x, float y, float z, StreamOutput *stream);
    bool get_home_offset(float& x, float& y, float& z);

    bool translate_trim(StreamOutput *stream);

    void SCARA_ang_move(float theta, float psi, float z, float feedrate);

    float slow_rate;

    struct {
        bool           is_scara:1;
    };
};

#endif /* SCARACAL_H_ */
