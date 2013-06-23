/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "PrepareScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
#include "libs/SerialMessage.h"
using namespace std;

PrepareScreen::PrepareScreen(){
    // Children screens
//    this->extruder_screen = (new ExtruderScreen()  )->set_parent(this);
//    this->temp_screen     = (new TempScreen()      )->set_parent(this);
}

void PrepareScreen::on_enter(){
    this->panel->enter_menu_mode();
    this->panel->setup_menu(7, 4);  // 7 menu items, 4 lines
    this->refresh_screen();
}

void PrepareScreen::on_refresh(){
    if( this->panel->menu_change() ){
        this->refresh_screen();
    }
    if( this->panel->click() ){
        this->clicked_menu_entry(this->panel->menu_current_line());
    }
}

void PrepareScreen::refresh_screen(){
    this->refresh_menu();
}

void PrepareScreen::display_menu_line(uint16_t line){
    switch( line ){
        case 0: this->panel->lcd->printf("Back"           ); break;  
        case 1: this->panel->lcd->printf("Home All Axis"  ); break; 
        case 2: this->panel->lcd->printf("Set Home"       ); break; 
        case 3: this->panel->lcd->printf("Pre Heat"       ); break; 
        case 4: this->panel->lcd->printf("Cool Down"      ); break; 
        case 5: this->panel->lcd->printf("Extrude"        ); break; 
        case 6: this->panel->lcd->printf("Set Temperature"); break; 
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line){
    switch( line ){
        case 0: this->panel->enter_screen(this->parent           ); break;
        case 1: send_gcode("G28"); break;
        case 2: send_gcode("G92 X0 Y0 Z0"); break;
        case 3: this->preheat(); break;
        case 4: this->cooldown(); break;
        case 5: this->panel->enter_screen(this->extruder_screen  ); break;
        case 6: this->panel->enter_screen(this->temp_screen      ); break;
    }


}


void PrepareScreen::preheat() {
}

void PrepareScreen::cooldown() {
}

void PrepareScreen::send_gcode(const char* gcstr) {
    string gcode(gcstr);
    struct SerialMessage message;
    message.message = gcode;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}
