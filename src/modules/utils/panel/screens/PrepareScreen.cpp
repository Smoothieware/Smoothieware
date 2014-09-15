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
#include "PrepareScreen.h"
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "checksumm.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "TemperatureControlPublicAccess.h"

#include <string>
using namespace std;

PrepareScreen::PrepareScreen()
{
    this->command = nullptr;

    // Children screens
    if(THEPANEL->temperature_screen != nullptr) {
        this->extruder_screen = (new ExtruderScreen())->set_parent(this);
        THEPANEL->temperature_screen->set_parent(this);
    }else{
        this->extruder_screen= nullptr;
    }
}

void PrepareScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    // if no heaters or extruder then don't show related menu items
    THEPANEL->setup_menu((THEPANEL->temperature_screen != nullptr) ? 9 : 5);
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
        case 1: THEPANEL->lcd->printf("Home All Axis"  ); break;
        case 2: THEPANEL->lcd->printf("Set Home"       ); break;
        case 3: THEPANEL->lcd->printf("Set Z0"         ); break;
        case 4: THEPANEL->lcd->printf("Motors off"     ); break;
        // these won't be accessed if no heaters or extruders
        case 5: THEPANEL->lcd->printf("Pre Heat"       ); break;
        case 6: THEPANEL->lcd->printf("Cool Down"      ); break;
        case 7: THEPANEL->lcd->printf("Extruder..."    ); break;
        case 8: THEPANEL->lcd->printf("Set Temperature"); break;
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
        case 1: command = "G28"; break;
        case 2: command = "G92 X0 Y0 Z0"; break;
        case 3: command = "G92 Z0"; break;
        case 4: command = "M84"; break;
        case 5: this->preheat(); break;
        case 6: this->cooldown(); break;
        case 7: THEPANEL->enter_screen(this->extruder_screen); break;
        case 8: THEPANEL->enter_screen(THEPANEL->temperature_screen); break;
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
    PublicData::set_value( temperature_control_checksum, hotend_checksum, &t );
    PublicData::set_value( temperature_control_checksum, bed_checksum, &t );
}

// queuing commands needs to be done from main loop
void PrepareScreen::on_main_loop()
{
    // change actual axis value
    if (this->command == nullptr) return;
    send_command(this->command);
    this->command = nullptr;
}
