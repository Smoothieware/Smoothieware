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
#include "PublicData.h"
#include "checksumm.h"
#include "LcdBase.h"
#include "JogScreenBasic.h"
#include "ExtruderScreen.h"
#include "JogScreen.h"

#include <math.h>
#include <stdio.h>

using namespace std;

#define NULL_CONTROL_MODE        0
#define AXIS_CONTROL_MODE        1
#define INCREMENT_SELECTION_MODE 2

JogScreenBasic::JogScreenBasic()
{
    this->command = nullptr;
    this->control_mode = NULL_CONTROL_MODE;

    this->jog_screen     = new JogScreen();
    this->jog_screen->set_parent(this);
    this->control_screen = new ControlScreen();
    this->control_screen->set_parent(this);
    // Children screens
    this->extruder_screen = (new ExtruderScreen())->set_parent(this);
}

void JogScreenBasic::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(7);
	get_current_pos(this->pos);
    this->refresh_menu();
    this->pos_changed = false;
}

void JogScreenBasic::on_refresh()
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

    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }

}

// queuing commands needs to be done from main loop
void JogScreenBasic::on_main_loop()
{
    // send gcode
    if (this->command != nullptr) {
        send_command(this->command);
        this->command = nullptr;
    } else if (this->pos_changed) {
        this->pos_changed = false;
        set_current_pos(this->controlled_axis, this->pos[this->controlled_axis - 'X']);
    } else {
        return;
    }

}

void JogScreenBasic::display_axis_line(char axis)
{
    THEPANEL->lcd->printf("Move %c    %8.3f", axis, this->pos[axis - 'X']);
}

void JogScreenBasic::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: THEPANEL->lcd->printf("Move X +/- 10mm  \x7E"); break;
        case 2: THEPANEL->lcd->printf("Move Y +/- 10mm  \x7E"); break;
        case 3: THEPANEL->lcd->printf("Move Z +/- 1mm   \x7E"); break;
        case 4: THEPANEL->lcd->printf("Extrude/Retract  \x7E"); break;
        case 5: THEPANEL->lcd->printf("Motors OFF       \x7E"); break;
        case 6: THEPANEL->lcd->printf("Advanced Jog     \x7E"); break;
    }
}


void JogScreenBasic::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: {
            THEPANEL->enter_screen(this->parent);
            return; }
        case 1: {
            this->set_jog_increment(10);
            this->enter_axis_control('X');
            return; }
        case 2: {
            this->set_jog_increment(10);
            this->enter_axis_control('Y');
            return; }
        case 3: {
            this->set_jog_increment(1);
            this->enter_axis_control('Z');
            return; }
        case 4: {
            THEPANEL->enter_screen(this->extruder_screen);
            return; }
        case 5: {
            command = "M84";
            return; }
        case 6: {
            THEPANEL->enter_screen(this->jog_screen);
            return;}
    }
}

void JogScreenBasic::enter_axis_control(char axis)
{
    this->control_mode = AXIS_CONTROL_MODE;
    this->controlled_axis = axis;
    THEPANEL->enter_control_mode(this->jog_increment, this->jog_increment / 10);
    THEPANEL->set_control_value(this->pos[axis - 'X']);
    THEPANEL->lcd->clear();
    THEPANEL->lcd->setCursor(0, 2);
    this->display_axis_line(this->controlled_axis);
}

void JogScreenBasic::enter_menu_control()
{
    this->control_mode = NULL_CONTROL_MODE;
    THEPANEL->enter_menu_mode();
}


void JogScreenBasic::set_current_pos(char axis, float p)
{
    // change pos by issuing a G0 Xnnn
    char buf[32];
    int n = snprintf(buf, sizeof(buf), "G0 %c%f F%d", axis, p, (int)round(THEPANEL->get_jogging_speed(axis)));
    string g(buf, n);
    send_gcode(g);
}
