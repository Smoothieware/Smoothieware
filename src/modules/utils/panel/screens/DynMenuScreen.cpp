/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "DynMenuScreen.h"
#include "libs/Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "LcdBase.h"
#include "Panel.h"
#include "ConfigValue.h"

#include <string.h>

#include <algorithm>

DynMenuScreen::DynMenuScreen()
{
    on_exit_cmd= nullptr;
    on_exit_fnc= nullptr;
}

DynMenuScreen::~DynMenuScreen()
{
    // free up the strdup() name and command
    for(auto &i : menu_items) {
        free(std::get<0>(i));
        if(std::get<3>(i)) free(std::get<2>(i)); // if it was a gcode
    }

    if(on_exit_cmd != nullptr) free(on_exit_cmd);
}

void DynMenuScreen::on_exit()
{
    if(on_exit_cmd != nullptr) {
        send_command(on_exit_cmd);
    }
    if(on_exit_fnc) {
        on_exit_fnc();
    }
    delete this;
}

void DynMenuScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(menu_items.size() + 1);
    this->refresh_menu();
}

void DynMenuScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void DynMenuScreen::display_menu_line(uint16_t line)
{
    if (line == 0) {
        THEPANEL->lcd->printf("Back");
    } else {
        line--;
        const char *name = std::get<0>(menu_items[line]);
        THEPANEL->lcd->printf("%-20s", name);
    }
}

void DynMenuScreen::clicked_menu_entry(uint16_t line)
{
    if (line == 0) {
        THEPANEL->enter_screen(this->parent);

    } else {
        if(std::get<3>(menu_items[line-1])) {
            // it is a gcode or command so must be executed in main loop
            send_command(std::get<2>(menu_items[line-1]));

        }else{
            // probably a new screen or function call
            std::get<1>(menu_items[line-1])();
        }
    }
}

void DynMenuScreen::addMenuItem(const MenuItemType& item)
{
    menu_items.push_back(item);
}

void DynMenuScreen::addMenuItem(const char *name, std::function<void(void)> func)
{
    string n(name);
    if(n.size() > 20) {
        n= n.substr(0, 20);
    }
    addMenuItem(make_tuple(strdup(n.c_str()), func, nullptr, false));
}

void DynMenuScreen::addMenuItem(const char *name, const char *gcode)
{
    string n(name);
    if(n.size() > 20) {
        n= n.substr(0, 20);
    }
    addMenuItem(make_tuple(strdup(n.c_str()), nullptr, strdup(gcode), true));
}

void DynMenuScreen::on_exit_action(const char *cmd)
{
    if(cmd == nullptr || strlen(cmd) == 0) {
        if(on_exit_cmd != nullptr) free(on_exit_cmd);
        on_exit_cmd= nullptr;
    }else{
        on_exit_cmd= strdup(cmd);
    }
}

void DynMenuScreen::on_exit_action(std::function<void()> fnc)
{
    on_exit_fnc= fnc;
}
