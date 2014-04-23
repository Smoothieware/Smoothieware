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
#include "Config.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "PublicData.h"
#include "Gcode.h"

#include "modules/robot/RobotPublicAccess.h"

#define return_error_on_unhandled_gcode_checksum    CHECKSUM("return_error_on_unhandled_gcode")

#define X_AXIS      0
#define Y_AXIS      1
#define Z_AXIS      2

ToolManager::ToolManager(){
    active_tool = 0;
}

void ToolManager::on_module_loaded(){
    this->on_config_reload(this);

    this->register_for_event(ON_CONFIG_RELOAD);
    this->register_for_event(ON_GCODE_RECEIVED);
}

void ToolManager::on_config_reload(void *argument){
    return_error_on_unhandled_gcode = THEKERNEL->config->value( return_error_on_unhandled_gcode_checksum )->by_default(false)->as_bool();
}

void ToolManager::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode*>(argument);

    if( gcode->has_letter('T') ){
        int new_tool = gcode->get_value('T');
        bool make_move = false;
        if ( gcode->has_letter('F') ){
            make_move = true;
        }
        gcode->mark_as_taken();
        if(new_tool >= (int)this->tools.size() || new_tool < 0){
            // invalid tool
            if( return_error_on_unhandled_gcode ) {
                char buf[32]; // should be big enough for any status
                int n= snprintf(buf, sizeof(buf), "T%d invalid tool ", new_tool);
                gcode->txt_after_ok.append(buf, n);
            }
        } else {
            if(new_tool != this->active_tool){
                void *returned_data;
                THEKERNEL->conveyor->wait_for_empty_queue();
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

                    this->tools[active_tool]->disable();
                    this->active_tool = new_tool;
                    this->tools[active_tool]->enable();

                    char buf[32];
                    snprintf(buf, 31, "G92 X%g Y%g Z%g", new_pos[X_AXIS], new_pos[Y_AXIS], new_pos[Z_AXIS]);
                    string s = buf;
                    Gcode *g = new Gcode(s, gcode->stream);
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, g);
                    delete g;

                    if(make_move){
                        //move to old position
                        snprintf(buf, 31, "G0 X%g Y%g Z%g", current_pos[X_AXIS], current_pos[Y_AXIS], current_pos[Z_AXIS]);
                        s = buf;
                        g = new Gcode(s, gcode->stream);
                        THEKERNEL->call_event(ON_GCODE_RECEIVED, g);
                        delete g;
                    }
                }
            }
        }
    }
}

// Add a tool to the tool list
void ToolManager::add_tool(Tool* tool_to_add){
    if(this->tools.size() == 0){
        tool_to_add->enable();
    } else {
        tool_to_add->disable();
    }
    this->tools.push_back( tool_to_add );
}



