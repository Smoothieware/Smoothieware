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


ExtruderScreen::ExtruderScreen(){
}

void ExtruderScreen::on_enter(){
    this->panel->enter_menu_mode();
    this->panel->setup_menu(3);  // 3 menu items, 4 lines
    this->refresh_screen();
}

void ExtruderScreen::on_refresh(){
    if( this->panel->menu_change() ){
        this->refresh_screen();
    }
    if( this->panel->click() ){
        this->clicked_menu_entry(this->panel->menu_current_line());
    }
}

void ExtruderScreen::refresh_screen(){
    this->refresh_menu();
}

void ExtruderScreen::display_menu_line(uint16_t line){
    switch( line ){
        case 0: this->panel->lcd->printf("Back");  break;  
        case 1: this->panel->lcd->printf("Extrude 5mm"); break;  
        case 2: this->panel->lcd->printf("Retract 5mm");  break;  
    }
}

void ExtruderScreen::clicked_menu_entry(uint16_t line){
    switch( line ){
        case 0: this->panel->enter_screen(this->parent); return;
        case 1: send_command("G91"); send_command("G1 E5");  send_command("G90"); break;
        case 2: send_command("G91"); send_command("G1 E-5"); send_command("G90"); break;
    }
}

void ExtruderScreen::send_command(const char* gcstr) {
    string cmd(gcstr);
    struct SerialMessage message;
    message.message = cmd;
    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
}
