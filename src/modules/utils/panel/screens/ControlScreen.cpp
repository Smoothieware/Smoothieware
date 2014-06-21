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
#include "ControlScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
#include "modules/robot/RobotPublicAccess.h"
#include "PublicData.h"
#include "checksumm.h"

#include <math.h>

using namespace std;

#define NULL_CONTROL_MODE        0
#define AXIS_CONTROL_MODE        1
#define INCREMENT_SELECTION_MODE 2

ControlScreen::ControlScreen()
{
    this->control_mode = NULL_CONTROL_MODE;
}

void ControlScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(4);
    get_current_pos(this->pos);
    this->refresh_menu();
    this->pos_changed = false;
}

// called in on_idle()
void ControlScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if (this->control_mode == AXIS_CONTROL_MODE) {

        if ( THEPANEL->click() ) {
            this->enter_menu_control();
            this->refresh_menu();

        } else if (THEPANEL->control_value_change()) {
            this->pos[this->controlled_axis - 'X'] = THEPANEL->get_control_value();
            THEPANEL->lcd->setCursor(0, 2);
            this->display_axis_line(this->controlled_axis);
            this->pos_changed = true; // make the gcode in main_loop
        }

    } else {
        if ( THEPANEL->click() ) {
            this->clicked_menu_entry(THEPANEL->get_menu_current_line());
        }
    }
}

// queuing gcodes needs to be done from main loop
void ControlScreen::on_main_loop()
{
    // change actual axis value
    if (!this->pos_changed) return;
    this->pos_changed = false;

    set_current_pos(this->controlled_axis, this->pos[this->controlled_axis - 'X']);
}

void ControlScreen::display_menu_line(uint16_t line)
{
    // in menu mode
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: this->display_axis_line('X'); break;
        case 2: this->display_axis_line('Y'); break;
        case 3: this->display_axis_line('Z'); break;
    }
}

void ControlScreen::display_axis_line(char axis)
{
    THEPANEL->lcd->printf("Move %c    %8.3f", axis, this->pos[axis - 'X']);
}


void ControlScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent   ); break;
        case 1: this->enter_axis_control('X'); break;
        case 2: this->enter_axis_control('Y'); break;
        case 3: this->enter_axis_control('Z'); break;
    }
}

void ControlScreen::enter_axis_control(char axis)
{
    this->control_mode = AXIS_CONTROL_MODE;
    this->controlled_axis = axis;
    THEPANEL->enter_control_mode(this->jog_increment, this->jog_increment / 10);
    THEPANEL->set_control_value(this->pos[axis - 'X']);
    THEPANEL->lcd->clear();
    THEPANEL->lcd->setCursor(0, 2);
    this->display_axis_line(this->controlled_axis);
}

void ControlScreen::enter_menu_control()
{
    this->control_mode = NULL_CONTROL_MODE;
    THEPANEL->enter_menu_mode();
}

void ControlScreen::get_current_pos(float *cp)
{
    void *returned_data;

    bool ok = PublicData::get_value( robot_checksum, current_position_checksum, &returned_data );
    if (ok) {
        float *p = static_cast<float *>(returned_data);
        cp[0] = p[0];
        cp[1] = p[1];
        cp[2] = p[2];
    }
}

void ControlScreen::set_current_pos(char axis, float p)
{
    // change pos by issuing a G0 Xnnn
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "G0 %c%f F%d", axis, p, (int)round(THEPANEL->get_jogging_speed(axis)));
    string g(buf, n);
    send_gcode(g);
}
