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
#include "TemperatureControl.h"
#include "PlayerPublicAccess.h"
#include "Player.h"
#include "ExtruderPublicAccess.h"
#include "PublicData.h"
#include "Robot.h"
#include "modules/robot/Conveyor.h"

#include "version.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Config.h"

#include <math.h>
#include <vector>

#define reporter_checksum          CHECKSUM("reporter")
#define enable_checksum            CHECKSUM("enable")

Reporter::Reporter(){}

Reporter::~Reporter(){}

void Reporter::on_module_loaded(){
  // Exit if this module is not enabled
  if ( !THEKERNEL->config->value( reporter_checksum, enable_checksum )->by_default(false)->as_bool() ) {
      delete this;
      return;
  }

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
                              // Find the status
                              //I=idle, P=printing from SD card, S=stopped (i.e. needs a reset),
                              //C=running config file, A=paused, D=pausing, R=resuming, B=busy (running a macro)
                              gcode->stream->printf("{\"status\":\"");
                              if(THEKERNEL->conveyor->is_queue_empty()){
                                gcode->stream->printf("I");
                              }else{
                                gcode->stream->printf("P");
                              }
                              if (THEKERNEL->is_halted()){
                                gcode->stream->printf("S");
                              }
                              void *returned_data;
                              if (PublicData::get_value( player_checksum, is_suspended_checksum, &returned_data )) {
                                bool b = *static_cast<bool *>(returned_data);
                                if(b){
                                  gcode->stream->printf("A");
                                }
                              }
                              gcode->stream->printf("\",\"heaters{{\":[");
                              std::vector<struct pad_temperature> controllers;
                              if (PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers)) {
                                for (auto &c : controllers) {
                                  if(&c == &controllers.back()){
                                    printf("%f,", c.current_temperature);
                                  }else{
                                    printf("%f,", c.current_temperature);
                                  }
                                }

                                gcode->stream->printf("],\"active\":[");
                                for (auto &c : controllers) {
                                  if(&c == &controllers.back()){
                                    printf("%f", c.target_temperature);
                                  }else{
                                    printf("%f,", c.target_temperature);
                                  }
                                }

                              // Smoothieware has no concept of a standby temperature.
                              gcode->stream->printf("],\"standby\":[0,0,0],");

                              // Heater status
                              // status of the heaters, 0=off, 1=standby, 2=active, 3=fault
                              gcode->stream->printf("\"hstat\":[");
                              for(auto &c : controllers){
                                // off
                                if(c.target_temperature == 0){
                                  gcode->stream->printf("0");
                                }
                                // active
                                if(c.target_temperature>c.current_temperature){
                                  gcode->stream->printf("2");
                                }
                                if(&c != &controllers.back()){
                                  gcode->stream->printf(",");
                                }
                              }
                            }

                              // Get position
                              gcode->stream->printf("]\"pos\":[");
                              float pos[3];
                              THEKERNEL->robot->get_axis_position(pos);
                              gcode->stream->printf("%f,%f,%f", pos[0], pos[1], pos[2]);

                              // End
                              //Extruder information
                              gcode->stream->printf("],\"extr\":[");
                              float *rd;
                              if(PublicData::get_value( extruder_checksum, (void **)&rd )){
                                gcode->stream->printf("%f", *(rd+5));
                              } else {
                                gcode->stream->printf("0");
                              }
                              gcode->stream->printf("],\"sfactor\":100.00,\"efactor\":[100.00,100.00],\"tool\":1,");
                              gcode->stream->printf("\"probe\":\"535\",\"fanRPM\":0,\"homed\":[0,0,0],\"fraction_printed\":");
                              // Get fraction printed
                              void *completed_data;
                              if (PublicData::get_value( player_checksum, get_progress_checksum, &completed_data )) {
                                struct pad_progress p =  *static_cast<struct pad_progress *>(completed_data);
                                gcode->stream->printf("%d",p.percent_complete);
                              }else{
                                gcode->stream->printf("0");
                              }

                              // JSon must have an EOL
                              gcode->stream->printf("}\n");
                }
            }
        }
    }
}
