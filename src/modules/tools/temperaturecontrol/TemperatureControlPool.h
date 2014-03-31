/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURECONTROLPOOL_H
#define TEMPERATURECONTROLPOOL_H

#include <math.h>
using namespace std;
#include <vector>

class TemperatureControl;
class PID_Autotuner;

#define temperature_control_checksum CHECKSUM("temperature_control")
#define enable_checksum              CHECKSUM("enable")

class TemperatureControlPool : public Module {
    public:
        TemperatureControlPool();

        void on_module_loaded();

        vector<TemperatureControl*> controllers;
        PID_Autotuner* PIDtuner;
};



#endif
