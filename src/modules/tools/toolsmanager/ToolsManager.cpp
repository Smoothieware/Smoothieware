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
#include "ToolsManager.h"
#include "Conveyor.h"

ToolsManager::ToolsManager(){
    active_tool = 0;
}

void ToolsManager::on_module_loaded(){
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GCODE_EXECUTE);
}

void ToolsManager::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('T') ){
        int new_tool = gcode->get_value('T');
        gcode->mark_as_taken();
        if(new_tool > this->tools.size() || new_tool < 0){
            // invalid tool
            gcode->stream->printf(";;T%d invalid tool\r\n", new_tool);
        } else {
            // pass along to on_gcode_execute
            THEKERNEL->conveyor->append_gcode(gcode);
        }
    }
}

// Compute extrusion speed based on parameters and gcode distance of travel
void ToolsManager::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('T') ){
        int new_tool = gcode->get_value('T');
        bool make_move = false;
        if ( gcode->has_letter('F') ){
            make_move = true;
        }
        if(new_tool != this->active_tool){
            void *returned_data;
            bool ok = THEKERNEL->public_data->get_value( robot_checksum, current_position_checksum, &returned_data );
            if(ok){
                // save current position to return to after applying extruder offset
                float *pos = static_cast<float *>(returned_data);
                float current_pos[3];
                for(int i=0;i<3;i++){
                    current_pos[i] = pos[i];
                }
                // update virtual tool position to the offset of the new tool and select it as active
                float new_pos[3];
                float *active_tool_offset = tools[this->active_tool]->get_offset();
                float *new_tool_offset = tools[new_tool]->get_offset();
                for(int i=0;i<3;i++){
                    new_pos[i] = current_pos[i] - active_tool_offset[i] + new_tool_offset[i];
                }

                this->active_tool = new_tool;
                ok = THEKERNEL->public_data->set_value( robot_checksum, current_position_checksum, new_pos );
                if(ok && make_move){
                    //TODO:move to old position (stored in current_pos[])
                }
            }
        }
    }
}

// Add a tool to the tool list
void ToolsManager::add_tool(Tool* tool_to_add){
    this->tools.push_back( tool_to_add );
}



