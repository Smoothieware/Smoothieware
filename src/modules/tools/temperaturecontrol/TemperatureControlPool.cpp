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
#include "TemperatureControlPool.h"
#include "TemperatureControl.h"

TemperatureControlPool::TemperatureControlPool(){}

void TemperatureControlPool::on_module_loaded(){

    vector<uint16_t> modules;
    this->kernel->config->get_module_list( &modules, temperature_control_checksum );

    for( int i = 0; i < modules.size(); i++ ){
        // If module is enabled
        if( this->kernel->config->value(temperature_control_checksum, modules[i], enable_checksum )->as_bool() == true ){
            TemperatureControl* controller = new TemperatureControl(modules[i]);
            this->kernel->add_module(controller); 
            this->controllers.push_back( controller );
        }
    }

}





