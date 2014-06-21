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
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>

using namespace std;


ExtruderScreen::ExtruderScreen()
{
}

void ExtruderScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(3);
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
    }
}

void ExtruderScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: command = "G91\nG1 E5 F100\nG90"; break;
        case 2: command = "G91\nG1 E-5 F100\nG90"; break;
    }
}

// queuing commands needs to be done from main loop
void ExtruderScreen::on_main_loop()
{
    if (this->command.empty()) return;
    send_command(this->command.c_str());
    this->command.clear();
}
