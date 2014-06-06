/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "ModifyValuesScreen.h"
#include "libs/Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "LcdBase.h"
#include "Panel.h"
#include "ConfigValue.h"

#include <string.h>

#include <algorithm>

using namespace std;

#define MENU_CONTROL_MODE 0
#define VALUE_CONTROL_MODE 1

ModifyValuesScreen::ModifyValuesScreen()
{
    execute_function = -1;
    control_mode = MENU_CONTROL_MODE;
}

ModifyValuesScreen::~ModifyValuesScreen()
{
    // free up the strdup() name
    for(auto i : menu_items) {
        free(std::get<0>(i));
    }
}

void ModifyValuesScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(menu_items.size() + 1);
    this->refresh_menu();
}

void ModifyValuesScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }

    if (this->control_mode == VALUE_CONTROL_MODE) {

        if ( THEPANEL->click() ) {
            // done changing value
            this->new_value = THEPANEL->get_control_value();
            execute_function= selected_item; // this causes on_main_loop to change the value
            this->control_mode = MENU_CONTROL_MODE;
            THEPANEL->enter_menu_mode(true);

        } else if (THEPANEL->control_value_change()) {
            float value = THEPANEL->get_control_value();
            if(!isnan(this->min_value) && value < this->min_value) {
                THEPANEL->set_control_value((value = this->min_value));
                THEPANEL->reset_counter();
            }
            if(!isnan(this->max_value) && value > this->max_value) {
                THEPANEL->set_control_value((value = this->max_value));
                THEPANEL->reset_counter();
            }
            THEPANEL->lcd->setCursor(0, 2);
            THEPANEL->lcd->printf("%10.3f    ", value);
        }

    } else {
        if ( THEPANEL->click() ) {
            this->clicked_menu_entry(THEPANEL->get_menu_current_line());
        }
    }
}

void ModifyValuesScreen::display_menu_line(uint16_t line)
{
    if (line == 0) {
        THEPANEL->lcd->printf("Back");
    } else {
        line--;
        const char *name = std::get<0>(menu_items[line]);
        float value = std::get<1>(menu_items[line])();
        THEPANEL->lcd->printf("%-10s %7.2f", name, value);
    }
}

void ModifyValuesScreen::clicked_menu_entry(uint16_t line)
{
    if (line == 0) {
        THEPANEL->enter_screen(this->parent);

    } else {
        line--;
        this->selected_item = line;
        this->control_mode = VALUE_CONTROL_MODE;

        const char *name = std::get<0>(menu_items[line]);
        float value = std::get<1>(menu_items[line])();
        float inc= std::get<3>(menu_items[line]);
        THEPANEL->enter_control_mode(inc, inc / 10);
        this->min_value= std::get<4>(menu_items[line]);
        this->max_value= std::get<5>(menu_items[line]);

        THEPANEL->set_control_value(value);
        THEPANEL->lcd->clear();
        THEPANEL->lcd->setCursor(0, 0);
        THEPANEL->lcd->printf("%s", name);
        THEPANEL->lcd->setCursor(0, 2);
        THEPANEL->lcd->printf("%10.3f", value);
    }
}

// queuing commands needs to be done from main loop
void ModifyValuesScreen::on_main_loop()
{
    // issue command
    if (execute_function == -1) return;

    // execute the setter function for the specified menu item
    std::get<2>(menu_items[execute_function])(this->new_value);
    execute_function = -1;
}

void ModifyValuesScreen::addMenuItem(const char *name, std::function<float()> getter, std::function<void(float)> setter, float inc, float  min, float max)
{
    string n(name);
    if(n.size() > 10) {
        n= n.substr(0, 10);
    }
    menu_items.push_back(make_tuple(strdup(n.c_str()), getter, setter, inc, min, max));
}
