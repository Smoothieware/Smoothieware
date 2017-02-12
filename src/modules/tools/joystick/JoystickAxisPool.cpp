/*
This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include <vector>
#include "JoystickAxisPool.h"
#include "JoystickAxis.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define joystick_checksum CHECKSUM("joystick")
#define enable_checksum CHECKSUM("enable")

void JoystickAxisPool::load_tools()
{
    //initialize a vector of the module name checksums
    vector<uint16_t> modules;

    //get a list of all the joystick module names defined in the config, store their checksums in modules
    THEKERNEL->config->get_module_list(&modules, joystick_checksum);

    //for all the joystick module names
    for (unsigned int i = 0; i < modules.size(); i++) {
        //if module is enabled
        if (THEKERNEL->config->value(joystick_checksum, modules[i], enable_checksum)->as_bool() == true) {
            //add a new joystick module to the kernel with this module name (checksum)
            JoystickAxis *controller = new JoystickAxis(modules[i]);
            THEKERNEL->add_module(controller);
        }
    }
}