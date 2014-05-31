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
#include "MainMenuScreen.h"
#include "JogScreen.h"
#include "ControlScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>

using namespace std;


JogScreen::JogScreen()
{
    this->control_screen = new ControlScreen();
    this->control_screen->set_parent(this);
}

void JogScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(4);
    this->refresh_menu();
}

void JogScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void JogScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: THEPANEL->lcd->printf("Move 10.0mm      \x7E"); break;
        case 2: THEPANEL->lcd->printf("Move  1.0mm      \x7E");  break;
        case 3: THEPANEL->lcd->printf("Move  0.1mm      \x7E");  break;
    }
}

void JogScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: this->control_screen->set_jog_increment(10.0); break;
        case 2: this->control_screen->set_jog_increment(1.0); break;
        case 3: this->control_screen->set_jog_increment(0.1); break;
    }
    THEPANEL->enter_screen(this->control_screen);
}
