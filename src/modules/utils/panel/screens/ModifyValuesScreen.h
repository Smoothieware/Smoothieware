/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MODIFYVALUESSCREEN_H
#define MODIFYVALUESSCREEN_H

#include "PanelScreen.h"

#include <string>
#include <vector>
#include <tuple>
#include <functional>
#include <cmath>

class ModifyValuesScreen : public PanelScreen
{
public:
    ModifyValuesScreen();
    virtual ~ModifyValuesScreen();

    void on_refresh();
    void on_enter();
    void on_main_loop();
    void display_menu_line(uint16_t line);
    void clicked_menu_entry(uint16_t line);
    int idle_timeout_secs(){ return 60; }

    typedef std::tuple<char *, std::function<float()>, std::function<void(float)>, const float (*)[3]> MenuItemType;
    // declare inc_min_max in the line in front of addMenuItem call as follows:
    // static const float my_imm[3] = {1.0F, NAN, NAN};
    // an allocated array won't be free'd!
    // std_values of inc_min_max: inc = 1.0F, min = NAN, max = NAN
    void addMenuItem(const char *name, std::function<float()> getter, std::function<void(float)> setter, const float (*inc_min_max)[3]);


private:
    int execute_function;
    float new_value, min_value, max_value;
    char control_mode;
    int selected_item;
    // name, getter function, setter function, increment_minimum_maximum
    std::vector<MenuItemType> menu_items;
};

#endif
