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
#include "SwitchPublicAccess.h"
#include "ToolManagerPublicAccess.h"
#include "NetworkPublicAccess.h"
#include "StringStream.h"
#include "Robot.h"
#include "ZProbe.h"
#include "modules/robot/Conveyor.h"
#include "utils.h"
#include "version.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Config.h"
#include "DirHandle.h"

#include <math.h>
#include <vector>
#include <algorithm>

#define reporter_checksum          CHECKSUM("reporter")
#define enable_checksum            CHECKSUM("enable")
#define machine_name_checksum      CHECKSUM("machine_name")
#define probe_height_checksum      CHECKSUM("probe_height")

// arm solutions
#define  arm_solution_checksum               CHECKSUM("arm_solution")
#define  cartesian_checksum                  CHECKSUM("cartesian")
#define  rotatable_cartesian_checksum        CHECKSUM("rotatable_cartesian")
#define  rostock_checksum                    CHECKSUM("rostock")
#define  linear_delta_checksum               CHECKSUM("linear_delta")
#define  rotary_delta_checksum               CHECKSUM("rotary_delta")
#define  delta_checksum                      CHECKSUM("delta")
#define  hbot_checksum                       CHECKSUM("hbot")
#define  corexy_checksum                     CHECKSUM("corexy")
#define  corexz_checksum                     CHECKSUM("corexz")
#define  kossel_checksum                     CHECKSUM("kossel")
#define  morgan_checksum                     CHECKSUM("morgan")

const string sd_root = "/sd";

static bool less_than_padtemp(const pad_temperature& leftSide, const pad_temperature& rightSide)
{
    // TODO: Deal correctly with heated chamber. Once PanelDue does as well.

    if (leftSide.designator == rightSide.designator)
        return leftSide.id < rightSide.id;

    return leftSide.designator == "B";
}

bool is_playing()
{
    void *returned_data;

    bool ok = PublicData::get_value(player_checksum, is_playing_checksum, &returned_data);

    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return b;
    }
    return false;
}

static bool is_suspended()
{
    void *returned_data;

    bool ok = PublicData::get_value(player_checksum, is_suspended_checksum, &returned_data);

    if (ok) {
        bool b = *static_cast<bool *>(returned_data);
        return b;
    }

    return false;
}

static char get_status_character()
{
    // Find the status
    // I=idle, P=printing from SD card, S=stopped (i.e. needs a reset),
    // C=running config file, A=paused, D=pausing, R=resuming, B=busy (running a macro)

    if (THEKERNEL->is_halted()) // Stopped
        return 'S';

    if (is_playing()) // Printing
        return 'P';

    if (is_suspended()) // Paused
        return 'A';

    if (!THEKERNEL->conveyor->is_idle()) // Printing
        return 'B';

    // Idle
    return 'I';
}

static float get_fan_percent()
{
    struct pad_switch s;
    bool ok = PublicData::get_value(switch_checksum, fan_checksum, 0, &s);

    if (ok) {
        if (s.state) {
            // TODO: figure out why this does not work. This always retuns 100.0.
            // This always returns 100.0 because s.value is not the current fan
            // speed, but the max_pwm. See Switch.cpp. Switch public access does
            // not send current fan speed. Requires change to Switch.cpp.
            return (100.0F * s.value) / 255.0F;
        }
    }

    return 0.0;
}

static int get_active_tool()
{
    void *returned_data;
    bool ok = PublicData::get_value(tool_manager_checksum, get_active_tool_checksum, &returned_data);

    if (ok) {
        int active_tool = *static_cast<int *>(returned_data);
        return active_tool;
    } else {
        return 0;
    }
}

static std::string get_probe_state()
{
    // TODO: replace with less wasteful code. E.g., through public message from zProbe.
    StringStream string_stream;
    Gcode gcode("M119", &string_stream);

    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);

    static std::string probe_label = "Probe: ";

    std::size_t probe_start = string_stream.getOutput().find(probe_label);

    if (probe_start == std::string::npos)
        return " NA";

    return string_stream.getOutput().substr(probe_start + probe_label.length());
}

void json_ls_command(string path, StreamOutput *stream)
{
    // If the S2 parameter is used on RepRapFirmware, then the file list is returned in JSON format
    // as a single array called "files" with each name that corresponds to a subdirectory preceded
    // by an asterisk, and the directory is returned in variable "dir". Example:
    //     M20 S2 P/gcodes
    //     {"dir":"\/gcodes","files":["4-piece-1-2-3-4.gcode","Hinged_Box.gcode","*Calibration pieces"]}

    // Removing this, as it makes use of the "current working directory" value in the KERNEL."
    // Seems risky as it cannot be considered to relate to the reported only. I.e., it might be changed
    // through other communications with the KERNEL.
    //    path = absolute_from_relative(path);


    std::string full_path = sd_root + path;

    stream->printf("{\"dir\":\"%s\"", path.c_str());
    stream->printf(",\"files\":");

    char ch = '[';

    DIR *d;
    struct dirent *p;
    d = opendir(full_path.c_str());

    if (d != NULL) {
        while ((p = readdir(d)) != NULL) {
            stream->printf("%c\"%s%s\"", ch, ((p->d_isdir) ? "*" : ""), lc(string(p->d_name)).c_str());
            ch = ',';
        }

        closedir(d);
    }

    stream->printf((ch == '[') ? "[]}\r\n" : "]}\r\n");
}

void json_file_info(string file, StreamOutput *stream)
{
    // M36 filename.gco
    // Returns information for the specified SD card file in JSON format. A sample response is:
    // {"err":0,"size":457574,"height":4.00,"layerHeight":0.25,"filament":[6556.3],
    //   "generatedBy":"Slic3r 1.1.7 on 2014-11-09 at 17:11:32"}
    //  The "err" field is zero if successful, nonzero if the file was not found or an error occurred
    // while processing it. The "size" field should always be present if the operation was successful.
    // The presence or absence of other fields depends on whether the corresponding values could be found
    // by reading the file.
    // The "filament" field is an array of the filament lengths required from each spool. The size is in
    // bytes, all other values are in mm. The fields may appear in any order, and additional fields may be
    // present.
    // If the file name parameter is not supplied and a file on the SD card is currently being printed,
    // then information for that file is returned including additional field "fileName".
    // This feature is used by the web interface and by PanelDue, so that if a connection is made when a
    // file is already being printed, the name and other information about that file can be shown.

    // See comment in json_ls_command
    // file = absolute_from_relative(file);

    std::string full_path_to_file = sd_root + file;

    stream->printf("{\"err\":%d", 0);
    stream->printf(",\"size\":%d", 12345);
    stream->printf(",\"height\":%f", 40.00);
    stream->printf(",\"layerHeight\":%f", 0.20);
    stream->printf(",\"filament\":[%f]", 123.4);

    stream->printf(",\"generatedBy\":\"%s\"", full_path_to_file.c_str());
    // TODO: remove me
    // hijack for debuging file path output
    // stream->printf(",\"generatedBy\":\"%s\"", "<Not implemented>");

    stream->printf("}\r\n");
}

Reporter::Reporter()
{
}

Reporter::~Reporter()
{
}

std::string geometry, myName;

void Reporter::on_module_loaded()
{
    // Exit if this module is not enabled
    if (!THEKERNEL->config->value(reporter_checksum, enable_checksum)->by_default(false)->as_bool()) {
        delete this;
        return;
    }

    // geometry should be one of "cartesian", "delta", "corexy, "corexz" etc.

    int solution_checksum = get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());

    if (solution_checksum == hbot_checksum || solution_checksum == corexy_checksum)
        geometry = "corexy";
    else if (solution_checksum == corexz_checksum)
        geometry = "corexz";
    else if (solution_checksum == rostock_checksum || solution_checksum == kossel_checksum || solution_checksum == delta_checksum || solution_checksum == linear_delta_checksum)
        geometry = "delta";
    else if (solution_checksum == rotatable_cartesian_checksum)
        geometry = "cartesian";     // not clear if this is OK.
    else if (solution_checksum == rotary_delta_checksum)
        geometry = "delta";         // not clear if this is OK.
    else if (solution_checksum == morgan_checksum)
        geometry = "morgan";
    else if (solution_checksum == cartesian_checksum)
        geometry = "cartesian";
    else
        geometry = "cartesian";

    myName = THEKERNEL->config->value(reporter_checksum, machine_name_checksum)->by_default("Smoothie")->as_string();

    std::replace(myName.begin(), myName.end(), '_', ' ');

    if (myName.empty())
        myName = "Smoothie";

    this->register_for_event(ON_GCODE_RECEIVED);
}

// TODO: remove once we repond properly to 'R' option.
static int line_count = 0;

void Reporter::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (gcode->has_m && gcode->m == 408 && gcode->has_letter('S')) {
        char ch;
        int s_value = static_cast<int>(gcode->get_value('S'));
        int tool_count = 0;

        switch (s_value) {
        case 0:
        case 1: {
            gcode->stream->printf("Getting M408 S%d status ", s_value);

            // Beginning
            gcode->stream->printf("{\"status\":\"%c\"", get_status_character());

            gcode->stream->printf(",\"heaters\":");

            std::vector<struct pad_temperature> controllers;

            bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);

            if (ok) {
                // The order expected to be returned is the bed then the extruders.
                // TODO: Deal correctly with heated chamber. Once PanelDue does as well.
                std::sort(controllers.begin(), controllers.end(), less_than_padtemp);
                ch = '[';

                for (auto &c : controllers) {
                    gcode->stream->printf("%c%.1f", ch, c.current_temperature);

                    if (c.designator != "B")
                        tool_count++;

                    ch = ',';
                }
                gcode->stream->printf((ch == '[') ? "[]" : "]");


                gcode->stream->printf(",\"active\":");

                ch = '[';

                for (auto &c : controllers) {
                    gcode->stream->printf("%c%.1f", ch, c.target_temperature);
                    ch = ',';
                }
                gcode->stream->printf((ch == '[') ? "[]" : "]");

                // Smoothieware has no concept of a standby temperature.
                // gcode->stream->printf(",\"standby\":[]");

                // status of the heaters, 0=off, 1=standby, 2=active, 3=fault
                gcode->stream->printf(",\"hstat\":");

                ch = '[';

                for (auto &c : controllers) {
                    // Fault (default)
                    char tmp = '3';

                    if (c.target_temperature == 0)
                        // Off
                        tmp = '0';
                    else if (c.target_temperature > 0)
                        // Active
                        tmp = '2';

                    gcode->stream->printf("%c%c", ch, tmp);
                    ch = ',';
                }

                gcode->stream->printf((ch == '[') ? "[]" : "]");
            }

            // Get position
            float pos[3];
            THEKERNEL->robot->get_axis_position(pos);
            gcode->stream->printf(",\"pos\":[%.2f,%.2f,%.2f]", pos[0], pos[1], pos[2]);

            // Extruder information

            // Potential set-up code. Have not figured this out yet.
            // pad_extruder_t rd;
            // bool extruder_data_ok = PublicData::get_value( extruder_checksum, (void *)&rd ));

            gcode->stream->printf(",\"extr\":");
            ch = '[';

            for (int i = 0; i < tool_count; i++) {
                // TODO: loop over active extruders and send extruder total extrusion since power up, last G92 or last M23
                // THEROBOT->get_axis_position(motor_id)??
                // stepper_motor->get_current_position()??
                gcode->stream->printf("%c%d", ch, 0);
                ch = ',';
            }

            gcode->stream->printf((ch == '[') ? "[]" : "]");

            // Send speed factor
            gcode->stream->printf(",\"sfactor\":%.2f", 6000.0F / THEKERNEL->robot->get_seconds_per_minute());

            // Send extruder override factors
            // TODO: loop over active extruders and send extruder override factor
            // gcode->stream->printf(",\"efactor\":");
            // ch = '[';
            //
            // for (int i = 0; i < tool_count; i++) {
            //
            //     gcode->stream->printf("%c%d", ch, 0);
            //     ch = ',';
            // }
            //
            // gcode->stream->printf((ch == '[') ? "[]" : "]");

            // Tool index
            gcode->stream->printf(",\"tool\":%d", get_active_tool());

            // Probe height
            gcode->stream->printf(",\"probe\":\"%s\"", get_probe_state().c_str());

            // Fan Percent
            gcode->stream->printf(",\"fanPercent\":%.2f", get_fan_percent());

            // Fan RPM
            // TODO:  fix fan RPM output
            // gcode->stream->printf(",\"fanRPM\":%d", 0);

            // Homed
            // TODO: fix homed status
            // the homed status of the X, Y and Z axes (or towers on a delta).
            //    0=axis has not been homed so position is not reliable,
            //    1=axis has been homed so position is reliable.
            gcode->stream->printf(",\"homed\":[1,1,1]");

            // Fraction printed
            gcode->stream->printf(",\"fraction_printed\":");

            // Get fraction printed
            void *completed_data;

            if (PublicData::get_value(player_checksum, get_progress_checksum, &completed_data)) {
                struct pad_progress p = *static_cast<struct pad_progress *>(completed_data);
                gcode->stream->printf("%.4f", ((float)p.percent_complete) / 100.0F);
            } else {
                gcode->stream->printf("0.0000");
            }

            if (s_value == 1) {             // also output static values
                // Printer name
                gcode->stream->printf(",\"myName\":\"%s\"", myName.c_str());

                // Printer geometry
                // one of "cartesian", "delta", "corexy, "corexz" etc.
                gcode->stream->printf(",\"geometry\":\"%s\"", geometry.c_str());

                gcode->stream->printf(",\"firmwareName\":\"Smoothie\"");

                gcode->stream->printf(",\"numTools\":%d", tool_count);
            }

            // TODO: replace with proper solution for:
            // seq:     the sequence number of the most recent non-trivial G-code response or error message.
            //          Only present if the R parameter was provided and the current sequence number is greater than that.
            // resp:    the most recent non-trivial G-code response or error message.
            //          Only present if the R parameter was provided and the current sequence number is greater.
            if (gcode->has_letter('R')) {
                gcode->stream->printf(",\"seq\":%d", line_count++);
                gcode->stream->printf(",\"resp\":\"%s\"", "<Smoothie: not yet implemented>");
            }

            // JSon must have an EOL
            gcode->stream->printf("}\r\n");
        } break;

        case 20: {
            std::string dir;

            if (gcode->has_letter('P')) {
                string possible_command = gcode->get_command();

                size_t beginning = possible_command.find_first_of("P");

                if (beginning != string::npos)
                    dir = possible_command.substr(beginning + 1);
            }

            if (dir == "")
                dir = "/";
                
            json_ls_command(dir, gcode->stream);
        }   break;

        case 36: {
            string possible_command = gcode->get_command();

            size_t beginning = possible_command.find_first_of("P");

            if (beginning != string::npos)
                json_file_info(possible_command.substr(beginning + 1), gcode->stream);

        } break;
        }
    }
}
