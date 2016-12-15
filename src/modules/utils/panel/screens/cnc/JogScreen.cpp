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
#include "MainMenuScreen.h"
#include "JogScreen.h"
#include "DirectJogScreen.h"
#include "ControlScreen.h"
#include "ModifyValuesScreen.h"
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
    THEPANEL->setup_menu(10);
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
        case 1: THEPANEL->lcd->printf("Move 10.0       "); break;
        case 2: THEPANEL->lcd->printf("Move  1.0       "); break;
        case 3: THEPANEL->lcd->printf("Move  0.1       "); break;
        case 4: THEPANEL->lcd->printf("Move  0.01      "); break;
        case 5: THEPANEL->lcd->printf("Move  0.001     "); break;
        case 6: THEPANEL->lcd->printf("MPG mode        "); break;
        case 7: THEPANEL->lcd->printf("Feed Rates      "); break;
        case 8: THEPANEL->lcd->printf("Goto Park Posn  "); break;
        case 9: THEPANEL->lcd->printf("Set Park Posn   "); break;
    }
}

void JogScreen::clicked_menu_entry(uint16_t line)
{
    bool dojog= true;
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: this->control_screen->set_jog_increment(10.0F); break;
        case 2: this->control_screen->set_jog_increment(1.0F); break;
        case 3: this->control_screen->set_jog_increment(0.1F); break;
        case 4: this->control_screen->set_jog_increment(0.01F); break;
        case 5: this->control_screen->set_jog_increment(0.001F); break;
        case 6: {
            auto djs = new DirectJogScreen();
            djs->set_parent(this);
            THEPANEL->enter_screen(djs); // self deleting
            dojog= false;
        } break;
        case 7:
            set_feed_rates();
            dojog= false;
        case 8: send_command("G28"); dojog= false; break;
        case 9: send_command("G28.1"); dojog= false; break;

    }

    if(dojog) {
        THEPANEL->enter_screen(this->control_screen);
    }
}

void JogScreen::set_feed_rates()
{
    auto mvs = new ModifyValuesScreen(true); // self delete on exit
    mvs->set_parent(this);

    for (int i = 0; i < 3; ++i) {
        string label;
        label.append(1, 'X' + i).append(" mm/min");
        mvs->addMenuItem(label.c_str(),
            [i]() -> float { return THEPANEL->get_jogging_speed(i); },
            [i](float v) { THEPANEL->set_jogging_speed(i, v); },
            10.0F, 1.0F);
    }

    THEPANEL->enter_screen(mvs);
}
