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
#include "ZProbe.h"
#include "modules/robot/Conveyor.h"

#include "version.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Config.h"

#include <math.h>
#include <vector>
#include <algorithm>

#define reporter_checksum          CHECKSUM("reporter")
#define enable_checksum            CHECKSUM("enable")
#define probe_height_checksum      CHECKSUM("probe_height")


bool less_than_padtemp(const pad_temperature& leftSide, const pad_temperature& rightSide)
{
        if(leftSide.designator == "B" && rightSide.designator == "T")
        {
                return true;
        }else if(leftSide.designator == rightSide.designator) {
                return (leftSide.id < rightSide.id);
        }

        return false;
};

Reporter::Reporter(){
}

Reporter::~Reporter(){
}

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
        if (gcode->has_m && gcode->m == 408 && gcode->has_letter('S')) {

                switch(static_cast<int>(gcode->get_value('S'))) {
                case 0:
                        gcode->stream->printf("Getting M408 S0 status");
                        // Beginning
                        // Find the status
                        //I=idle, P=printing from SD card, S=stopped (i.e. needs a reset),
                        //C=running config file, A=paused, D=pausing, R=resuming, B=busy (running a macro)
                        gcode->stream->printf("{\"status\":\"");
                        if(THEKERNEL->conveyor->is_queue_empty()) {
                                gcode->stream->printf("I");
                        }else{
                                gcode->stream->printf("P");
                        }
                        if (THEKERNEL->is_halted()) {
                                gcode->stream->printf("S");
                        }
                        void *returned_data;
                        if (PublicData::get_value( player_checksum, is_suspended_checksum, &returned_data )) {
                                bool b = *static_cast<bool *>(returned_data);
                                if(b) {
                                        gcode->stream->printf("A");
                                }
                        }

                        gcode->stream->printf("\",\"heaters\":[");
                        std::vector<struct pad_temperature> controllers;
                        bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
                        if (ok) {
                                // The order expected to be returned is the bed then the extruders.
                                std::sort(controllers.begin(), controllers.end(), less_than_padtemp);
                                for (unsigned i=0; i<controllers.size(); i++) {
                                        //gcode->stream->printf("designator = %s : id = %d\n", controllers[i].designator.c_str(), controllers[i].id);
                                        if(i == controllers.size()-1) {
                                                gcode->stream->printf("%f", controllers[i].current_temperature);
                                        }else{
                                                gcode->stream->printf("%f,", controllers[i].current_temperature);
                                        }
                                }

                                gcode->stream->printf("],\"active\":[");
                                for (unsigned i=0; i<controllers.size(); i++) {
                                        if(i == controllers.size()-1) {
                                                gcode->stream->printf("%f", controllers[i].target_temperature);
                                        }else{
                                                gcode->stream->printf("%f,", controllers[i].target_temperature);
                                        }
                                }

                                // Smoothieware has no concept of a standby temperature.
                                gcode->stream->printf("],\"standby\":[],");
                                // status of the heaters, 0=off, 1=standby, 2=active, 3=fault
                                gcode->stream->printf("\"hstat\":[");
                                if(controllers.size() > 0) {
                                        for(unsigned i=0; i<controllers.size(); i++) {
                                                // off
                                                if(controllers[i].target_temperature == 0) {
                                                        gcode->stream->printf("0");
                                                }else if(controllers[i].target_temperature > 0) {
                                                        gcode->stream->printf("2");
                                                }
                                                if(i != controllers.size()-1) {
                                                        gcode->stream->printf(",");
                                                }
                                        }
                                }
                        }

                        // Get position
                        gcode->stream->printf("],\"pos\":[");
                        float pos[3];
                        THEKERNEL->robot->get_axis_position(pos);
                        gcode->stream->printf("%f,%f,%f", pos[0], pos[1], pos[2]);

                        // End
                        //Extruder information
                        gcode->stream->printf("],\"extr\":[");
                        float *rd;
                        if(PublicData::get_value( extruder_checksum, (void **)&rd )) {
                                gcode->stream->printf("%f", *(rd+5));
                        } else {
                                gcode->stream->printf("0");
                        }
                        gcode->stream->printf("],\"sfactor\":");
                        gcode->stream->printf("%f", 6000.0F / THEKERNEL->robot->get_seconds_per_minute());
                        gcode->stream->printf(",\"efactor\":[");
                        if(PublicData::get_value( extruder_checksum, (void **)&rd )) {
                                gcode->stream->printf("%f", *(rd+2)*100.0F);
                        }
                        // Tool index
                        gcode->stream->printf("],\"tool\":1,");

                        // Probe height
                        gcode->stream->printf("\"probe\":\"");
                        // This seems to crash Smoothieware.....
                        // if(THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(false)->as_bool() )
                        // {
                        //   gcode->stream->printf("%f\n", THEKERNEL->config->value(zprobe_checksum, probe_height_checksum)->by_default(5.0F)->as_number());
                        // }
                        gcode->stream->printf("\",\"fanRPM\":0,\"homed\":[0,0,0],\"fraction_printed\":");
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
