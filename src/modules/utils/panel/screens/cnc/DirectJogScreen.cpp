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

#include <math.h>
#include <stdio.h>
#include <string>

using namespace std;

DirectJogScreen::DirectJogScreen()
{
    mode= AXIS_SELECT;
}

void DirectJogScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(4);
    get_current_pos(this->pos);
    this->refresh_menu();
    this->pos_changed = false;
    mode= AXIS_SELECT;
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
                this->enter_menu_control();
                this->refresh_menu();
                break;

            case AXIS_SELECT:
                this->clicked_menu_entry(THEPANEL->get_menu_current_line());
                break;
        }

    } else if(pos_changed) {
        pos_changed= false;
        // get real time positions
        ActuatorCoordinates current_position{
            THEKERNEL->robot->actuators[X_AXIS]->get_current_position(),
            THEKERNEL->robot->actuators[Y_AXIS]->get_current_position(),
            THEKERNEL->robot->actuators[Z_AXIS]->get_current_position()
        };

        // get machine position from the actuator position using FK
        float mpos[3];
        THEKERNEL->robot->arm_solution->actuator_to_cartesian(current_position, mpos);
        Robot::wcs_t wpos= THEKERNEL->robot->mcs2wcs(mpos);
        this->pos[0]= THEKERNEL->robot->from_millimeters(std::get<X_AXIS>(wpos));
        this->pos[1]= THEKERNEL->robot->from_millimeters(std::get<Y_AXIS>(wpos));
        this->pos[2]= THEKERNEL->robot->from_millimeters(std::get<Z_AXIS>(wpos));

        THEPANEL->lcd->setCursor(0, 2);
        this->display_axis_line(this->axis);
    }

}

void DirectJogScreen::display_menu_line(uint16_t line)
{
    // in menu mode
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: this->display_axis_line(X_AXIS); break;
        case 2: this->display_axis_line(Y_AXIS); break;
        case 3: this->display_axis_line(Z_AXIS); break;
    }
}

void DirectJogScreen::display_axis_line(uint8_t axis)
{
    THEPANEL->lcd->printf("%c    %8.3f", 'X' + axis, this->pos[axis]);
}


void DirectJogScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
        case 1: this->enter_axis_control(X_AXIS); break;
        case 2: this->enter_axis_control(Y_AXIS); break;
        case 3: this->enter_axis_control(Z_AXIS); break;
    }
}

void DirectJogScreen::on_exit()
{
    delete this;
}

// encoder tick, called in an interrupt everytime we get an encoder tick
void DirectJogScreen::tick(int change)
{
    int steps= std::abs(change); // TODO may need multiplier here for regular encoder, or do speed check and increase rate accordingly
    for (int i = 0; i < steps; ++i) {
        // foreach tick we issue a step to the selected axis
        if(i > 0) wait_us(10); // fastest rate is 100KHz
        THEKERNEL->robot->actuators[axis]->manual_step(change < 0);
    }

    pos_changed= true;
}

void DirectJogScreen::enter_axis_control(uint8_t axis)
{
    this->mode = JOG;
    this->axis = axis;
    THEPANEL->lcd->clear();
    THEPANEL->lcd->setCursor(0, 0);
    THEPANEL->lcd->printf("Encoder Jog Control");
    THEPANEL->lcd->setCursor(0, 2);
    this->display_axis_line(this->axis);

    using std::placeholders::_1;
    THEPANEL->enter_direct_encoder_mode(std::bind(&DirectJogScreen::tick, this, _1));
}

void DirectJogScreen::enter_menu_control()
{
    this->mode = AXIS_SELECT;
    THEPANEL->enter_menu_mode();
}
