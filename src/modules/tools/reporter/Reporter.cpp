/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "Reporter.h"
#include "Gcode.h"
#include "StreamOutput.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "TemperatureControl.h"
#include "PublicData.h"
#include "Robot.h"

#include <math.h>
#include <vector>


Reporter::Reporter(){}

Reporter::~Reporter(){}

void Reporter::on_module_loaded(){
    this->register_for_event(ON_GCODE_RECEIVED);
}

void Reporter::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (gcode->has_m) {
        if( gcode->m == 408 ) {
            if (gcode->has_letter('S')) {
                switch(static_cast<int>(gcode->get_value('S'))){
                        case 0:
                              // Beginning
                              gcode->stream->printf("{\"status\":\"I\",");
                              gcode->stream->printf("\"heaters{{\":[");
                              for(auto it = THEKERNEL->temperature_control_pool->get_controllers().begin();
                                      it != THEKERNEL->temperature_control_pool->get_controllers().end();
                                      it++) {
                                        auto & id = *it;
                                TemperatureControl *temp_control;
                                void *returned_data;
                                bool ok = PublicData::get_value( temperature_control_checksum, pool_index_checksum, id, &returned_data );

                                if (ok) {
                                    temp_control =  *static_cast<TemperatureControl **>(returned_data);

                                } else {
                                    gcode->stream->printf("No temperature control with index %d found\r\n", id);
                                    return;
                                }
                                    if(std::next(it) == THEKERNEL->temperature_control_pool->get_controllers().end()){
                                      printf("%f", temp_control->get_temperature());
                                    }else{
                                      printf("%f,", temp_control->get_temperature());
                                    }
                              }

                              gcode->stream->printf("],\"active\":[101,102,103],\"standby\":[202,404,606],\"hstat\":[0,2,1],\"pos\":[");

                              // Get position
                              float pos[3];
                              THEKERNEL->robot->get_axis_position(pos);
                              gcode->stream->printf("%f,%f,%f", pos[0], pos[1], pos[2]);

                              // End
                              gcode->stream->printf("],\"extr\":[12,34],\"sfactor\":100.00,\"efactor\":[100.00,100.00],\"tool\":1,\"probe\":\"535\",\"fanRPM\":0,\"homed\":[0,0,0],\"fraction_printed\":0.572}\n");
                }
            }
        }
    }
}
