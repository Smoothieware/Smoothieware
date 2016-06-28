/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "DirectJogScreen.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "MainMenuScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Robot.h"
#include "PublicData.h"
#include "checksumm.h"
#include "LcdBase.h"
#include "StepperMotor.h"
#include "BaseSolution.h"
#include "mbed.h"

#include <math.h>
#include <stdio.h>
#include <string>

using namespace std;

DirectJogScreen::DirectJogScreen(){}

void DirectJogScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    mode= MULTIPLIER;
    THEPANEL->setup_menu(7);
    this->refresh_menu();
    this->pos_changed = false;
    get_actuator_pos();
}

// called in on_idle()
void DirectJogScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if ( THEPANEL->click() ) {
        switch(mode) {
            case JOG:
                THEROBOT->reset_position_from_current_actuator_position(); // needed as we were changing the actuator only
                this->enter_menu_control();
                this->refresh_menu();
                break;

            case MULTIPLIER:
            case AXIS_SELECT:
                this->clicked_menu_entry(THEPANEL->get_menu_current_line());
                break;
        }

    } else if(pos_changed) {
        pos_changed= false;
        get_actuator_pos();
        THEPANEL->lcd->setCursor(0, 2);
        this->display_axis_line(this->axis);
    }

}

void DirectJogScreen::display_menu_line(uint16_t line)
{
    // in menu mode
    if(mode == MULTIPLIER) {
        switch ( line ) {
            case 0: THEPANEL->lcd->printf("Back");  break;
            case 1: THEPANEL->lcd->printf("x1"); break;
            case 2: THEPANEL->lcd->printf("x4"); break;
            case 3: THEPANEL->lcd->printf("x8"); break;
            case 4: THEPANEL->lcd->printf("x16"); break;
            case 5: THEPANEL->lcd->printf("x32"); break;
            case 6: THEPANEL->lcd->printf("x64"); break;
        }

    }else if(mode == AXIS_SELECT){
        switch ( line ) {
            case 0: THEPANEL->lcd->printf("Back");  break;
            case 1: this->display_axis_line(X_AXIS); break;
            case 2: this->display_axis_line(Y_AXIS); break;
            case 3: this->display_axis_line(Z_AXIS); break;
        }
    }
}

void DirectJogScreen::display_axis_line(uint8_t axis)
{
    THEPANEL->lcd->printf("%c    %8.3f", 'X' + axis, this->pos[axis]);
}


void DirectJogScreen::clicked_menu_entry(uint16_t line)
{
    if(mode == MULTIPLIER) {
        switch ( line ) {
            case 0: THEPANEL->enter_screen(this->parent); break;
            case 1: multiplier= 1; break;
            case 2: multiplier= 4; break;
            case 3: multiplier= 8; break;
            case 4: multiplier= 16; break;
            case 5: multiplier= 32; break;
            case 6: multiplier= 64; break;
        }

        if(line > 0) {
            enter_menu_control();
        }

    }else if(mode == AXIS_SELECT){
        switch ( line ) {
            case 0: THEPANEL->enter_screen(this->parent); break;
            case 1: this->enter_axis_control(X_AXIS); break;
            case 2: this->enter_axis_control(Y_AXIS); break;
            case 3: this->enter_axis_control(Z_AXIS); break;
        }
    }
}

void DirectJogScreen::on_exit()
{
    // we need to update the positions after manually ticking
    THEROBOT->reset_position_from_current_actuator_position();
    delete this;
}

void DirectJogScreen::get_actuator_pos()
{
        // get real time positions
        ActuatorCoordinates current_position{
            THEROBOT->actuators[X_AXIS]->get_current_position(),
            THEROBOT->actuators[Y_AXIS]->get_current_position(),
            THEROBOT->actuators[Z_AXIS]->get_current_position()
        };

        // get machine position from the actuator position using FK
        float mpos[3];
        THEROBOT->arm_solution->actuator_to_cartesian(current_position, mpos);
        Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
        this->pos[0]= THEROBOT->from_millimeters(std::get<X_AXIS>(wpos));
        this->pos[1]= THEROBOT->from_millimeters(std::get<Y_AXIS>(wpos));
        this->pos[2]= THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));
}

// encoder tick, called in an interrupt everytime we get an encoder tick
// fastest is 200pulses/sec
void DirectJogScreen::tick(int change)
{
    for (int i = 0; i < multiplier; ++i) {
        if(i != 0) wait_us(100); // if you go faster than this you miss encoder changes
        THEROBOT->actuators[axis]->manual_step(change < 0);
    }
    pos_changed= true;
}

void DirectJogScreen::enter_axis_control(uint8_t axis)
{
    this->mode = JOG;
    this->axis = axis;
    THEPANEL->lcd->clear();
    THEPANEL->lcd->setCursor(0, 0);
    THEPANEL->lcd->printf("MPG mode x%d", multiplier);
    THEPANEL->lcd->setCursor(0, 2);
    get_actuator_pos();
    this->display_axis_line(this->axis);

    using std::placeholders::_1;
    THEPANEL->enter_direct_encoder_mode(std::bind(&DirectJogScreen::tick, this, _1));
}

void DirectJogScreen::enter_menu_control()
{
    this->mode = AXIS_SELECT;
    THEPANEL->setup_menu(4);
    THEPANEL->enter_menu_mode(true);
}
