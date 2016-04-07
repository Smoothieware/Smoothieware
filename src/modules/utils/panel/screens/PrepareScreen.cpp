/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "MainMenuScreen.h"
#include "WatchScreen.h"
#include "FileScreen.h"
#include "JogScreen.h"
#include "JogScreenBasic.h"
#include "ControlScreen.h"
#include "PrepareScreen.h"
#include "ProbeScreen.h"
#include "CalibrateScreen.h"
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "checksumm.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "TemperatureControlPublicAccess.h"
#include "Planner.h"
#include "ModifyValuesScreen.h"
#include "TemperatureControlPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "EndstopsPublicAccess.h"

#include <string>
using namespace std;

#define extruder_checksum CHECKSUM("extruder")

PrepareScreen::PrepareScreen()
{
    // Children screens
    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok && controllers.size() > 0) {
        this->extruder_screen = (new ExtruderScreen())->set_parent(this);
    }else{
        this->extruder_screen= nullptr;
    }
}
// setup and enter the configure screen
void PrepareScreen::setupConfigureScreen()
{
    auto mvs= new ModifyValuesScreen(true); // delete itself on exit
    mvs->set_parent(this);

    // acceleration
    mvs->addMenuItem("Acceleration", // menu name
        []() -> float { return THEKERNEL->planner->get_acceleration(); }, // getter
        [this](float acc) { send_gcode("M204", 'S', acc); }, // setter
        10.0F, // increment
        1.0F, // Min
        10000.0F // Max
        );
		
	mvs->addMenuItem("Extru Accel", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+3); else return 0; }, // getter
        [this](float acc) { send_gcode("M204", 'E', acc); }, // setter
        10.0F, // increment
        1.0F   // Min
        );
    mvs->addMenuItem("Filament diameter",
        // gets filament diameter for currently active extruder
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+1); else return 0.0F; },
        [this](float v) { send_gcode("M200", 'D', v); },
        0.01F,
        0.0F,
        4.0F,
		1.75F
        );

    // flow rate
    mvs->addMenuItem("Flow rate", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+2)*100.0F; else return 100.0F; }, // getter as fraction
        [this](float fr) { send_gcode("M221", 'S', fr); }, // setter in percent
        1.0F, // increment
        1.0F  // Min
        );



    mvs->addMenuItem("Retract len", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+4); else return 0; }, // getter
        [this](float l) { send_gcode("M207", 'S', l); }, // setter
        0.1F, // increment
        0.0F  // Min
        );
		
    // steps/mm
    mvs->addMenuItem("X steps/mm",
        []() -> float { return THEKERNEL->robot->actuators[0]->get_steps_per_mm(); },
        [](float v) { THEKERNEL->robot->actuators[0]->change_steps_per_mm(v); },
        0.1F,
        1.0F
        );

    mvs->addMenuItem("Y steps/mm",
        []() -> float { return THEKERNEL->robot->actuators[1]->get_steps_per_mm(); },
        [](float v) { THEKERNEL->robot->actuators[1]->change_steps_per_mm(v); },
        0.1F,
        1.0F
        );

    mvs->addMenuItem("Z steps/mm",
        []() -> float { return THEKERNEL->robot->actuators[2]->get_steps_per_mm(); },
        [](float v) { THEKERNEL->robot->actuators[2]->change_steps_per_mm(v); },
        0.1F,
        1.0F
        );
	mvs->addMenuItem("E steps/mm",
        // gets steps/mm for currently active extruder
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *rd; else return 0.0F; },
        [this](float v) { send_gcode("M92", 'E', v); },
        0.1F,
        1.0F
        );

    mvs->addMenuItem("Z Home Ofs",
        []() -> float { void *rd; PublicData::get_value( endstops_checksum, home_offset_checksum, &rd ); return rd==nullptr ? 0.0F : ((float*)rd)[2]; },
        [this](float v) { send_gcode("M206", 'Z', v); },
        0.01F
        );

    mvs->addMenuItem("LCD contrast",
        []() -> float { return THEPANEL->lcd->getContrast(); },
        [this](float v) { THEPANEL->lcd->setContrast(v); },
        1,
        0,
        255,
        true // instant update
        );

    THEPANEL->enter_screen(mvs);
}

void PrepareScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    // if no heaters or extruder then don't show related menu items
    THEPANEL->setup_menu(12);
    this->refresh_menu();
}

void PrepareScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void PrepareScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back"           ); break;
		case 1: THEPANEL->lcd->printf("Switches"); break;
        case 2: THEPANEL->lcd->printf("Home All Axis"  ); break;
        case 3: THEPANEL->lcd->printf("Preheat PLA 200"); break;
        case 4: THEPANEL->lcd->printf("Preheat ABS 245"); break;
        case 5: THEPANEL->lcd->printf("Flush Nozzles"  ); break;
        case 6: THEPANEL->lcd->printf("Adjust Temperature"); break;
        case 7: THEPANEL->lcd->printf("Cool Down"      ); break;
        case 8: THEPANEL->lcd->printf("Extrude/Retract"); break;
		case 9: THEPANEL->lcd->printf("Motors OFF"     ); break;
        case 10: THEPANEL->lcd->printf("Calibration"); break;
		case 11: THEPANEL->lcd->printf("Advanced Cfg"); break;
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
		case 1: THEPANEL->enter_screen(THEPANEL->custom_screen ); break;
        case 2: send_command("G28"); break;
        case 3: send_command("T1\nM104 S200\nG4 P1\nT0\nM104 S200\nM140 S60"); break;
        case 4: send_command("T1\nM104 S245\nG4 P1\nT0\nM104 S245\nM140 S100"); break;
		case 5: send_command("T0\nG92 E0\nG1 E35 F300\nG92 E0\nT1\nG92 E0\nG1 E35 F300\nG92 E0\nT0"); break;
		case 6: setup_temperature_screen(); break;
		case 7: this->cooldown(); break;
		case 8: THEPANEL->enter_screen(this->extruder_screen); break;
        case 9: send_command("M84"); break;
		case 10: THEPANEL->enter_screen((new CalibrateScreen())->set_parent(this)); break;
		case 11: setupConfigureScreen(); break;
    }
}

void PrepareScreen::preheat()
{
    float t = THEPANEL->get_default_hotend_temp();
    PublicData::set_value( temperature_control_checksum, hotend_checksum, &t );
    t = THEPANEL->get_default_bed_temp();
    PublicData::set_value( temperature_control_checksum, bed_checksum, &t );
}

void PrepareScreen::cooldown()
{
    float t = 0;
    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok) {
        for (auto &c : controllers) {
            PublicData::set_value( temperature_control_checksum, c.id, &t );
        }
    }
 }

static float getTargetTemperature(uint16_t heater_cs)
{
    struct pad_temperature temp;
    bool ok = PublicData::get_value( temperature_control_checksum, current_temperature_checksum, heater_cs, &temp );

    if (ok) {
        return temp.target_temperature;
    }

    return 0.0F;
}

void PrepareScreen::setup_temperature_screen()
{
    // setup temperature screen
    auto mvs= new ModifyValuesScreen(true); // delete itself on exit
    mvs->set_parent(this);

    int cnt= 0;
    // returns enabled temperature controllers
    std::vector<struct pad_temperature> controllers;
    bool ok = PublicData::get_value(temperature_control_checksum, poll_controls_checksum, &controllers);
    if (ok) {
        for (auto &c : controllers) {
            // rename if two of the known types
            const char *name;
            if(c.designator == "T") name= "Hotend";
            else if(c.designator == "Q") name= "Hotend2";
            else if(c.designator == "B") name= "Bed";
            else if(c.designator == "P") name= "PCB";
            else name= c.designator.c_str();
            uint16_t i= c.id;

            mvs->addMenuItem(name, // menu name
                [i]() -> float { return getTargetTemperature(i); }, // getter
                [i](float t) { PublicData::set_value( temperature_control_checksum, i, &t ); }, // setter
                1.0F, // increment
                0.0F, // Min
                500.0F // Max
            );
            cnt++;
        }
    }

    if(cnt > 0) {
        THEPANEL->enter_screen(mvs);
    }else{
        // no heaters and probably no extruders either
        delete mvs;
    }
}
