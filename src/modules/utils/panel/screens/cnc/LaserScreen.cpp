/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LaserScreen.h"

#include "Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "ModifyValuesScreen.h"
#include "DynMenuScreen.h"
#include "PublicData.h"
#include "checksumm.h"

#ifndef NO_TOOLS_LASER
#include "Laser.h"
#endif

#include <string>

using namespace std;

#define laser_checksum CHECKSUM("laser")

LaserScreen::LaserScreen()
{}

void LaserScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(3);
    this->refresh_menu();
}

void LaserScreen::on_exit()
{
    delete this; // self deleting
}

void LaserScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void LaserScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: THEPANEL->lcd->printf("Set Power Scale"); break;
        case 2: THEPANEL->lcd->printf("Test Fire");  break;
    }
}

void LaserScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: setPowerScreen(); break;
        case 2: testFireScreen(); break;
    }
}

void LaserScreen::testFireScreen()
{
    auto dms= new DynMenuScreen();  // self delete on exit
    dms->set_parent(this->parent); // because this will have been deleted when it exits
    dms->set_timeout(10); // 10 seconds and it will turn off
    dms->on_exit_action("fire off");

    dms->addMenuItem("Off", "fire off");
    dms->addMenuItem("Low", "fire 10");
    dms->addMenuItem("1/4", "fire 25");
    dms->addMenuItem("Half", "fire 50");
    dms->addMenuItem("3/4",  "fire 75");
    dms->addMenuItem("Full", "fire 100");
    dms->addMenuItem("Keep On", [dms]() { dms->set_timeout(600); dms->on_exit_action(""); }); // will leave laser on when menu exits

    THEPANEL->enter_screen(dms);
}

void LaserScreen::setPowerScreen()
{
    auto mvs= new ModifyValuesScreen(true);  // self delete on exit
    mvs->set_parent(this->parent); // because this will have been deleted when it exits

    #ifndef NO_TOOLS_LASER
    Laser *plaser= nullptr;
    if(PublicData::get_value(laser_checksum, (void *)&plaser) && plaser != nullptr) {
        mvs->addMenuItem("Power %",
            [plaser]() -> float { return plaser->get_scale(); },
            [plaser](float v) { plaser->set_scale(v); },
            1.0F,
            0.0F,
            200.0F,
            true // instant
            );
    }
    #endif

    THEPANEL->enter_screen(mvs);
}
