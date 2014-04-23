/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ExtruderMaker.h"
#include "Extruder.h"
#include "Config.h"
#include "ToolManager.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include <math.h>
using namespace std;
#include <vector>


ExtruderMaker::ExtruderMaker(){}

void ExtruderMaker::on_module_loaded(){

    // If there is a "single" extruder configured ( old config syntax from when there was only one extruder module, no pool/maker
    if( THEKERNEL->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() == true ){

        // Make a new extruder module
        Extruder* extruder = new Extruder(0);

        // Signal the extruder it will have to read config as an alone extruder
        extruder->single_config = true;

        // Add the module to the kernel
        THEKERNEL->add_module( extruder );

        // Add the module to the ToolManager
        THEKERNEL->toolmanager->add_tool( extruder );

    }

    // Get every "declared" extruder module ( new, multiextruder syntax )
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, extruder_checksum );

    // For every extruder found
    for( unsigned int i = 0; i < modules.size(); i++ ){

        // If module is enabled
        if( THEKERNEL->config->value(extruder_checksum, modules[i], enable_checksum )->as_bool() == true ){

            // Make a new extruder module
            Extruder* extruder = new Extruder(modules[i]);

            // Add the module to the kernel
            THEKERNEL->add_module( extruder );

            // Add the module to the ToolsManager
            THEKERNEL->toolmanager->add_tool( extruder );

        }

    }

}





