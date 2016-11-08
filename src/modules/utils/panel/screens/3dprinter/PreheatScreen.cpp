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
#include "PreheatScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "checksumm.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"

#include <string>
using namespace std;

PreheatScreen::PreheatScreen()
{
    this->hotend_temp_preheat = THEPANEL->get_default_hotend_temp();
    this->bed_temp_preheat = THEPANEL->get_default_bed_temp();
}

void PreheatScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(3);
    this->refresh_menu();
}

void PreheatScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void PreheatScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back"          ); break;
        case 1: THEPANEL->lcd->printf("Preheat hotend"); break;
        case 2: THEPANEL->lcd->printf("Preheat bed"   ); break;
    }
}

void PreheatScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
        case 1: preheat_hotend(); break;
        case 2: preheat_bed(); break;
    }
}

void PreheatScreen::preheat_hotend() {
    PublicData::set_value( temperature_control_checksum, hotend_checksum, &(this->hotend_temp_preheat) );
}

void PreheatScreen::preheat_bed()
{
    PublicData::set_value( temperature_control_checksum, bed_checksum, &(this->bed_temp_preheat) );
}
