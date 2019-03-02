/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ProbeScreen.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "StringStream.h"
#include "Gcode.h"

#include <string>

using namespace std;

ProbeScreen::ProbeScreen()
{
    this->do_probe= false;
    this->do_status= false;
    this->new_result= false;
}

void ProbeScreen::on_exit()
{
    this->do_probe= false;
    this->do_status= false;
    this->new_result= false;
    delete this;
}

void ProbeScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(3);
    this->refresh_menu();
}

void ProbeScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
    if(this->new_result) {
        this->new_result= false;
        for ( uint8_t l=0; (l < THEPANEL->get_screen_lines()-3) && (this->result.size() > l*20); l++ ) {
            THEPANEL->lcd->setCursor(0, l+3);
            THEPANEL->lcd->printf("%-20s", this->result.substr(l*20,20).c_str());
        }
    }
}

void ProbeScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back");  break;
        case 1: THEPANEL->lcd->printf("Status");  break;
        case 2: THEPANEL->lcd->printf("Z Probe");  break;
    }
}

void ProbeScreen::clicked_menu_entry(uint16_t line)
{
    this->do_status= false;
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1: this->do_status= true; this->tcnt= 1; break;
        case 2: this->do_probe= true; break;
    }
}

// queuing commands needs to be done from main loop
void ProbeScreen::on_main_loop()
{
    if (this->do_probe) {
        this->do_probe= false;
        StringStream string_stream;
        Gcode gcode("G30", &string_stream);
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
        this->result= string_stream.getOutput();
        this->new_result= true;

    }else if (this->do_status && --this->tcnt == 0) {
        // this will refresh the results every 10 main loop iterations
        this->tcnt= 10; // update every 10 times
        StringStream string_stream;
        Gcode gcode("M119", &string_stream);
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
        this->result= string_stream.getOutput();
        this->new_result= true;
    }
}
