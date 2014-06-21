/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "Gcode.h"
#include "LcdBase.h"
#include "libs/StreamOutput.h"

#include <string>
#include <vector>

using namespace std;

PanelScreen::PanelScreen() {}

void PanelScreen::on_refresh() {}
void PanelScreen::on_main_loop() {}

void PanelScreen::on_enter() {}

void PanelScreen::refresh_menu(bool clear)
{
    if (clear) THEPANEL->lcd->clear();
    for (uint16_t i = THEPANEL->menu_start_line; i < THEPANEL->menu_start_line + min( THEPANEL->menu_rows, THEPANEL->panel_lines ); i++ ) {
        THEPANEL->lcd->setCursor(2, i - THEPANEL->menu_start_line );
        this->display_menu_line(i);
    }
    THEPANEL->lcd->setCursor(0, THEPANEL->menu_current_line - THEPANEL->menu_start_line );
    THEPANEL->lcd->printf(">");
}

void PanelScreen::refresh_screen(bool clear)
{
    if (clear) THEPANEL->lcd->clear();
    for (uint16_t i = THEPANEL->menu_start_line; i < THEPANEL->menu_start_line + min( THEPANEL->menu_rows, THEPANEL->panel_lines ); i++ ) {
        THEPANEL->lcd->setCursor(0, i - THEPANEL->menu_start_line );
        this->display_menu_line(i);
    }
}

PanelScreen *PanelScreen::set_parent(PanelScreen *passed_parent)
{
    this->parent = passed_parent;
    return this;
}

// Helper for screens to send a gcode, must be called from main loop
void PanelScreen::send_gcode(std::string g)
{
    Gcode gcode(g, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode );
}

void PanelScreen::send_gcode(const char *gm_code, char parameter, float value)
{
    char buf[132];
    int n = snprintf(buf, sizeof(buf), "%s %c%f", gm_code, parameter, value);
    string g(buf, n);
    send_gcode(g);
}

// Helper to send commands, must be called from mainloop
// may contain multipe commands separated by \n
void PanelScreen::send_command(const char *gcstr)
{
    string cmd(gcstr);
    vector<string> q;
    while (cmd.size() > 0) {
        size_t b = cmd.find_first_of("\n");
        if ( b == string::npos ) {
            q.push_back(cmd);
            break;
        }
        q.push_back(cmd.substr( 0, b ));
        cmd = cmd.substr(b + 1);
    }

    // for each command send it
    for (std::vector<string>::iterator i = q.begin(); i != q.end(); ++i) {
        struct SerialMessage message;
        message.message = *i;
        message.stream = &(StreamOutput::NullStream);
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }
}
