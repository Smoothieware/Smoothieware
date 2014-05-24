/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TOUCHPROBE_H_
#define TOUCHPROBE_H_

#include "libs/Module.h"

class Touchprobe: public Module {
    private:
        void wait_for_touch(int distance[]);
        void flush_log();

        FILE*          logfile;
        string         filename;
        StepperMotor*  steppers[3];
        Pin            pin;
        unsigned int   debounce_count;

    public:
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_received(void* argument);
        void on_idle(void* argument);

        float         probe_rate;
        unsigned int   mcode;
        bool           enabled;
        bool           should_log;
};

#endif /* TOUCHPROBE_H_ */
