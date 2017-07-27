/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include <math.h>
using namespace std;
#include <vector>
#include "ReadPinPool.h"
#include "ReadPin.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define enable_checksum CHECKSUM("enable")

void ReadPinPool::load_tools() {
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list(&modules, readpin_checksum);

    for (const auto& m : modules) {
        if (THEKERNEL->config->value(readpin_checksum, m, enable_checksum )->as_bool()) {
            auto controller = new ReadPin(m);
            THEKERNEL->add_module(controller);
        }
    }
}





