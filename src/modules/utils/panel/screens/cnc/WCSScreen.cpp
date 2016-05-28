/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "WCSScreen.h"
#include "Panel.h"
#include "LcdBase.h"
#include "Robot.h"
#include "utils.h"

WCSScreen::WCSScreen()
{}

WCSScreen::~WCSScreen()
{}

void WCSScreen::on_exit()
{
    delete this;
}

void WCSScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(MAX_WCS+1);
    this->refresh_menu();
}

void WCSScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void WCSScreen::display_menu_line(uint16_t line)
{
    if (line == 0) {
        THEPANEL->lcd->printf("Back");
    } else {
        THEPANEL->lcd->printf("%-10s", wcs2gcode(line-1).c_str());
    }
}

void WCSScreen::clicked_menu_entry(uint16_t line)
{
    if (line > 0) {
        send_command(wcs2gcode(line-1).c_str());
    }
    THEPANEL->enter_screen(this->parent);
}
