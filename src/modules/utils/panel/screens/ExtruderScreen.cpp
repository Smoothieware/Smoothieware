/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ModifyValuesScreen.h"
#include "PublicData.h"
#include "checksumm.h"

#include <string>

using namespace std;

#define extruder_checksum CHECKSUM("extruder")

ExtruderScreen::ExtruderScreen()
{
    this->command= nullptr;
}

void ExtruderScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(4);
    this->refresh_menu();
}

void ExtruderScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void ExtruderScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: THEPANEL->lcd->printf("Extrude 5mm"); break;
        case 2: THEPANEL->lcd->printf("Retract 5mm");  break;
        case 3: THEPANEL->lcd->printf("Settings...");  break;
    }
}

void ExtruderScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: command = "G91\nG1 E5 F100\nG90"; break;
        case 2: command = "G91\nG1 E-5 F100\nG90"; break;
        case 3: setupConfigSettings(); break; // lazy load
    }
}

// queuing commands needs to be done from main loop
void ExtruderScreen::on_main_loop()
{
    if (this->command == nullptr) return;
    send_command(this->command);
    this->command= nullptr;
}

void ExtruderScreen::setupConfigSettings()
{
    auto mvs= new ModifyValuesScreen(true);  // self delete on exit
    mvs->set_parent(this);

    mvs->addMenuItem("E steps/mm",
        // gets steps/mm for currently active extruder
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *rd; else return 0.0F; },
        [this](float v) { send_gcode("M92", 'E', v); },
        0.1F,
        1.0F
        );

    mvs->addMenuItem("Filament diameter",
        // gets filament diameter for currently active extruder
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+1); else return 0.0F; },
        [this](float v) { send_gcode("M200", 'D', v); },
        0.01F,
        0.0F,
        4.0F
        );

    // flow rate
    mvs->addMenuItem("Flow rate", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+2)*100.0F; else return 100.0F; }, // getter as fraction
        [this](float fr) { send_gcode("M221", 'S', fr); }, // setter in percent
        1.0F, // increment
        1.0F  // Min
        );

    mvs->addMenuItem("Accel", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+3); else return 0; }, // getter
        [this](float acc) { send_gcode("M204", 'E', acc); }, // setter
        10.0F, // increment
        1.0F   // Min
        );

    mvs->addMenuItem("Retract len", // menu name
        []() -> float { float *rd; if(PublicData::get_value( extruder_checksum, (void **)&rd )) return *(rd+4); else return 0; }, // getter
        [this](float l) { send_gcode("M207", 'S', l); }, // setter
        0.1F, // increment
        0.0F  // Min
        );

    THEPANEL->enter_screen(mvs);
}
