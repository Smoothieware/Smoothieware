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
    this->probing= false;
    this->do_status= false;
    this->new_result= false;
}

void ProbeScreen::on_exit()
{
    this->do_probe= false;
    this->do_status= false;
    this->new_result= false;
    this->probing= false;
    delete this;
}

void ProbeScreen::on_enter()
{
    this->do_probe= false;
    this->probing= false;
    this->do_status= false;
    this->new_result= false;
    this->display_result= false;

    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(8);
    this->refresh_menu();
}

void ProbeScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if ( THEPANEL->click() ) {
        if(this->probing) {
            // abort
            THEKERNEL->call_event(ON_HALT, nullptr);
            display_result= true;
        }
        this->clicked_menu_entry(this->display_result ? 0 : THEPANEL->get_menu_current_line());
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
        case 2: THEPANEL->lcd->printf("X- Probe");  break;
        case 3: THEPANEL->lcd->printf("Y- Probe");  break;
        case 4: THEPANEL->lcd->printf("Z- Probe");  break;
        case 5: THEPANEL->lcd->printf("X+ Probe");  break;
        case 6: THEPANEL->lcd->printf("Y+ Probe");  break;
        case 7: THEPANEL->lcd->printf("Z+ Probe");  break;
    }
}

void ProbeScreen::clicked_menu_entry(uint16_t line)
{
    this->do_status= false;
    this->do_probe= false;
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); return;
        case 1:
            this->do_status= true;
            this->tcnt= 1;
            THEPANEL->lcd->clear();
            THEPANEL->lcd->setCursor(0, 0);
            THEPANEL->lcd->printf("Endstop/Probe Status... ");
            THEPANEL->lcd->setCursor(0, 1);
            THEPANEL->lcd->printf("Click to exit");
            break;
        case 2: this->do_probe= true; probe_dir= false; probe_axis= X_AXIS; break;
        case 3: this->do_probe= true; probe_dir= false; probe_axis= Y_AXIS; break;
        case 4: this->do_probe= true; probe_dir= false; probe_axis= Z_AXIS; break;
        case 5: this->do_probe= true; probe_dir= true; probe_axis= X_AXIS; break;
        case 6: this->do_probe= true; probe_dir= true; probe_axis= Y_AXIS; break;
        case 7: this->do_probe= true; probe_dir= true; probe_axis= Z_AXIS; break;
    }

    if(do_status || do_probe) THEPANEL->enter_nop_mode();
}

// queuing commands needs to be done from main loop
void ProbeScreen::on_main_loop()
{
    if (this->do_probe) {
        this->do_probe= false;
        StringStream string_stream;
        string cmd("G38.3 ");
        switch(probe_axis) {
            case X_AXIS: cmd.append(probe_dir ? "X50" : "X-50"); break;
            case Y_AXIS: cmd.append(probe_dir ? "Y50" : "Y-50"); break;
            case Z_AXIS: cmd.append(probe_dir ? "Z50" : "Z-50"); break;
        }

        THEPANEL->lcd->clear();
        THEPANEL->lcd->setCursor(0, 0);
        THEPANEL->lcd->printf("Probing... ");
        THEPANEL->lcd->setCursor(0, 1);
        THEPANEL->lcd->printf("Click to abort");
        this->probing= true;

        struct SerialMessage message;
        message.message = cmd;
        message.stream = &string_stream;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
        if(THEKERNEL->is_halted()) return;

        THEPANEL->lcd->clear();
        THEPANEL->lcd->setCursor(0, 0);
        THEPANEL->lcd->printf("Probing complete... ");
        THEPANEL->lcd->setCursor(0, 1);
        THEPANEL->lcd->printf("Click to exit");
        // TODO could use THEROBOT->get_last_probe_position() -> std::tuple<float, float, float, uint8_t>
        this->result= string_stream.getOutput();
        size_t p= this->result.find("PRB:");
        if(p != string::npos) {
            this->result= this->result.substr(p+4); // extract just coordinates
            size_t p= this->result.find("]");
            this->result= this->result.substr(0, p); // chop off end
        }
        this->new_result= true;
        this->probing= false;
        this->display_result= true;

    }else if (this->do_status && --this->tcnt == 0) {
        // this will refresh the results every 10 main loop iterations
        this->tcnt= 10; // update every 10 times
        StringStream string_stream;
        Gcode gcode("M119", &string_stream);
        THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
        this->result= string_stream.getOutput();
        this->new_result= true;
        this->display_result= true;
    }
}
