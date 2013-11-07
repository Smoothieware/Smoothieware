/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PANELSCREEN_H
#define PANELSCREEN_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "LcdBase.h"
#include "Panel.h"

class Panel;
class PanelScreen
{
public:
    PanelScreen();
    virtual void on_refresh();
    virtual void on_main_loop();
    PanelScreen *set_panel(Panel *parent);
    PanelScreen *set_parent(PanelScreen *passed_parent);
    virtual void on_enter();
    // if you completely rewrite the screen do not clear it, this avoids flicker
    void refresh_screen(bool clear);
    void refresh_menu(bool clear);
    void refresh_menu(void)
    {
        refresh_menu(true);
    };
    virtual void display_menu_line(uint16_t line) = 0;

    Panel *panel;
    PanelScreen *parent;

protected:
    void send_gcode(std::string g);
    void send_command(const char *gcstr);
};






#endif
