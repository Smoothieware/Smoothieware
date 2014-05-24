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
#include "StreamOutputPool.h"

#include <math.h>
using namespace std;
#include <vector>

#define extruder_module_enable_checksum      CHECKSUM("extruder_module_enable")
#define extruder_checksum                    CHECKSUM("extruder")
#define enable_checksum                      CHECKSUM("enable")

void ExtruderMaker::on_module_loaded(){

    // If there is a "single" extruder configured ( old config syntax from when there was only one extruder module, no pool/maker
    if( THEKERNEL->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() ){
        THEKERNEL->streams->printf("NOTE: Old Extruder config used\n");

        // Make a new extruder module
        Extruder* extruder = new Extruder(0, true);

        // Add the module to the kernel
        THEKERNEL->add_module( extruder );

        // no toolmanager required so do not create one
        return;
    }

    // Get every "declared" extruder module ( new, multiextruder syntax )
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, extruder_checksum );

    if(modules.size() == 0) {
        THEKERNEL->streams->printf("NOTE: No extruders configured\n");
        // no extruders
        delete this;
        return;
    }

    if(modules.size() == 1) {
        // only one extruder so no tool manager required
        // If module is enabled
        if( THEKERNEL->config->value(extruder_checksum, modules[0], enable_checksum )->as_bool() ){
            THEKERNEL->streams->printf("NOTE: One extruder configured\n");
            // Make a new extruder module
            Extruder* extruder = new Extruder(modules[0]);
            // Add the module to the kernel
            THEKERNEL->add_module( extruder );
            extruder->enable();

        }else{
            THEKERNEL->streams->printf("NOTE: No extruders enabled\n");
            delete this;
        }
        return;
    }

    // ONLY do this if multitool enabled and more than one tool is defined
    ToolManager *toolmanager= new ToolManager();
    THEKERNEL->add_module( toolmanager );

    // For every extruder found
    int cnt= 0;
    for( unsigned int i = 0; i < modules.size(); i++ ){
        // If module is enabled
        if( THEKERNEL->config->value(extruder_checksum, modules[i], enable_checksum )->as_bool() ){

            // Make a new extruder module
            Extruder* extruder = new Extruder(modules[i]);

            // Add the module to the kernel
            THEKERNEL->add_module( extruder );

            // Add the module to the ToolsManager
            toolmanager->add_tool( extruder );

            cnt++;
        }

    }
    THEKERNEL->streams->printf("NOTE: %d extruders enabled out of %d\n", cnt, modules.size());
}





