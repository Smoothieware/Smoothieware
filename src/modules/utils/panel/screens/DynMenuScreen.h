/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "PanelScreen.h"

#include <string>
#include <vector>
#include <tuple>
#include <functional>

class DynMenuScreen : public PanelScreen
{
public:
    DynMenuScreen();
    virtual ~DynMenuScreen();

    void on_refresh();
    void on_enter();
    void on_exit();

    void display_menu_line(uint16_t line);
    void clicked_menu_entry(uint16_t line);
    int idle_timeout_secs(){ return timeout; }
    void set_timeout(int n) { timeout= n; }
    void addMenuItem(const char *name, std::function<void()> fnc);
    void addMenuItem(const char *name, const char *gcode);
    void on_exit_action(const char *);
    void on_exit_action(std::function<void()>);

private:
    using MenuItemType = std::tuple<char *, std::function<void()>, char *, bool>;
    void addMenuItem(const MenuItemType& item);
    int timeout{60};

    // name, function, command, type
    std::vector<MenuItemType> menu_items;
    std::function<void()> on_exit_fnc;
    char *on_exit_cmd;
};
