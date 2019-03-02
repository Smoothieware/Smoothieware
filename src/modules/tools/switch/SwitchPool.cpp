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
#include "SwitchPool.h"
#include "Switch.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

#define switch_checksum CHECKSUM("switch")
#define enable_checksum CHECKSUM("enable")

void SwitchPool::load_tools()
{
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, switch_checksum );

    for( unsigned int i = 0; i < modules.size(); i++ ) {
        // If module is enabled
        if( THEKERNEL->config->value(switch_checksum, modules[i], enable_checksum )->as_bool() == true ) {
            Switch *controller = new Switch(modules[i]);
            THEKERNEL->add_module(controller);
        }
    }

}





