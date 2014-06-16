/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ZPROBE_H_
#define ZPROBE_H_

#include "Module.h"
#include "Pin.h"

class StepperMotor;
class Gcode;
class StreamOutput;

class ZProbe: public Module
{

public:
    void on_module_loaded();
    void on_config_reload(void *argument);
    void on_gcode_received(void *argument);
    void on_idle(void *argument);
    uint32_t acceleration_tick(uint32_t dummy);


private:
    bool wait_for_probe(int steps[3]);
    bool run_probe(int& steps, bool fast= false);
    bool probe_delta_tower(int& steps, float x, float y);
    bool return_probe(int steps);
    bool calibrate_delta_endstops(Gcode *gcode);
    bool calibrate_delta_radius(Gcode *gcode);
    void coordinated_move(float x, float y, float z, float feedrate, bool relative=false);
    void home();
    bool set_trim(float x, float y, float z, StreamOutput *stream);
    bool get_trim(float& x, float& y, float& z);

    float          probe_radius;
    float          probe_height;
    float          current_feedrate;
    float          slow_feedrate;
    float          fast_feedrate;
    Pin            pin;
    uint8_t        debounce_count;
    struct {
        bool           running:1;
        bool           is_delta:1;
    };
};

#endif /* ZPROBE_H_ */
