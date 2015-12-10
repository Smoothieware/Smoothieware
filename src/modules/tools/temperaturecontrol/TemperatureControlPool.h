/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TEMPERATURECONTROLPOOL_H
#define TEMPERATURECONTROLPOOL_H

#include <vector>

class TemperatureControlPool {
    public:
        void load_tools();
        const std::vector<uint16_t>& get_controllers() const { return controllers; };

    private:
        std::vector<uint16_t> controllers;
};



#endif
