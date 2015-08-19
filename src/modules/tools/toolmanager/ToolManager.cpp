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
#include "ToolManager.h"
#include "Tool.h"
#include "PublicDataRequest.h"
#include "ToolManagerPublicAccess.h"
#include "Config.h"
#include "Robot.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"

#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "FileStream.h"

#include "modules/robot/RobotPublicAccess.h"

#define return_error_on_unhandled_gcode_checksum    CHECKSUM("return_error_on_unhandled_gcode")

ToolManager::ToolManager(){
    active_tool = 0;
    current_tool_name = CHECKSUM("hotend");
}

void ToolManager::on_module_loaded(){
    this->on_config_reload(this);

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

void ToolManager::on_config_reload(void *argument){
    return_error_on_unhandled_gcode = THEKERNEL->config->value( return_error_on_unhandled_gcode_checksum )->by_default(false)->as_bool();
}

void ToolManager::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('T') ){
        int new_tool = gcode->get_value('T');
        if(new_tool >= (int)this->tools.size() || new_tool < 0){
            // invalid tool
            if( return_error_on_unhandled_gcode ) {
                char buf[32]; // should be big enough for any status
                int n= snprintf(buf, sizeof(buf), "T%d invalid tool ", new_tool);
                gcode->txt_after_ok.append(buf, n);
            }
        } else {
            if(new_tool != this->active_tool){
                // We must wait for an empty queue before we can disable the current extruder
                THEKERNEL->conveyor->wait_for_empty_queue();
                this->tools[active_tool]->disable();
                this->active_tool = new_tool;
                this->current_tool_name = this->tools[active_tool]->get_name();
                this->tools[active_tool]->enable();

                //send new_tool_offsets to robot
                const float *new_tool_offset = tools[new_tool]->get_offset();
                THEKERNEL->robot->setToolOffset(new_tool_offset);
            }
        }
    }
}

void ToolManager::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(tool_manager_checksum)) return;
    if(!pdr->second_element_is(is_active_tool_checksum)) return;

    // check that we control the given tool
    bool managed= false;
    for(auto t : tools) {
        uint16_t n= t->get_name();
        if(pdr->third_element_is(n)){
            managed= true;
            break;
        }
    }

    // we are not managing this tool so do not answer
    if(!managed) return;

    pdr->set_data_ptr(&this->current_tool_name);
    pdr->set_taken();
}

void ToolManager::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(tool_manager_checksum)) return;

    // ok this is targeted at us, so change tools
    //uint16_t tool_name= *static_cast<float*>(pdr->get_data_ptr());
    // TODO: fire a tool change gcode
    //pdr->set_taken();
}

// Add a tool to the tool list
void ToolManager::add_tool(Tool* tool_to_add){
    if(this->tools.size() == 0){
        tool_to_add->enable();
        this->current_tool_name = tool_to_add->get_name();
        //send new_tool_offsets to robot
        const float *new_tool_offset = tool_to_add->get_offset();
        THEKERNEL->robot->setToolOffset(new_tool_offset);
    } else {
        tool_to_add->disable();
    }
    this->tools.push_back( tool_to_add );
}



