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
#include "ModifyValuesScreen.h"
#include "WCSScreen.h"

#include <string>
using namespace std;

PrepareScreen::PrepareScreen()
{
}

void PrepareScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(11);
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
        case 1: THEPANEL->lcd->printf("Home All Axes"  ); break;
        case 2: THEPANEL->lcd->printf("Touch off X"    ); break;
        case 3: THEPANEL->lcd->printf("Touch off Y"    ); break;
        case 4: THEPANEL->lcd->printf("Touch off Z"    ); break;
        case 5: THEPANEL->lcd->printf("Spindle on"     ); break;
        case 6: THEPANEL->lcd->printf("Spindle off"    ); break;
        case 7: THEPANEL->lcd->printf("Motors off"     ); break;
        case 8: THEPANEL->lcd->printf("Coordinate system"); break;
        case 9: THEPANEL->lcd->printf("millimeters"    ); break;
        case 10: THEPANEL->lcd->printf("inches"        ); break;
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
        case 1: send_command("$H"); break;
        case 2: send_command("G10 L20 P0 X0"); break;
        case 3: send_command("G10 L20 P0 Y0"); break;
        case 4: send_command("G10 L20 P0 Z0"); break;
        case 5: send_command("M3"); break;
        case 6: send_command("M5"); break;
        case 7: send_command("M84"); break;
        case 8: THEPANEL->enter_screen((new WCSScreen())->set_parent(this)); break; // self deleteing
        case 9: send_command("G21"); break;
        case 10: send_command("G20"); break;
    }
}

