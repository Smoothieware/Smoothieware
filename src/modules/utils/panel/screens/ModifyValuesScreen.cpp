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

void ModifyValuesScreen::on_enter()
{
    this->panel->enter_menu_mode();
    this->panel->setup_menu(menu_items.size() + 1);
    this->refresh_menu();
}

void ModifyValuesScreen::on_refresh()
{
    if ( this->panel->menu_change() ) {
        this->refresh_menu();
    }

    if (this->control_mode == VALUE_CONTROL_MODE) {

        if ( this->panel->click() ) {
            // done changing value
            execute_function= selected_item; // this causes on_main_loop to change the value
            this->control_mode = MENU_CONTROL_MODE;
            this->panel->enter_menu_mode();
            this->refresh_menu();

        } else if (this->panel->control_value_change()) {
            this->new_value = this->panel->get_control_value();
            if(!isnan(this->min_value) && this->new_value < this->min_value) {
                this->panel->set_control_value((this->new_value = this->min_value));
                this->panel->reset_counter();
            }
            if(!isnan(this->max_value) && this->new_value > this->max_value) {
                this->panel->set_control_value((this->new_value = this->max_value));
                this->panel->reset_counter();
            }
            this->panel->lcd->setCursor(0, 2);
            this->panel->lcd->printf("%10.3f    ", this->new_value);
        }

    } else {
        if ( this->panel->click() ) {
            this->clicked_menu_entry(this->panel->get_menu_current_line());
        }
    }
}

void ModifyValuesScreen::display_menu_line(uint16_t line)
{
    if (line == 0) {
        this->panel->lcd->printf("Back");
    } else {
        line--;
        const char *name = std::get<0>(menu_items[line]);
        float value = std::get<1>(menu_items[line])();
        this->panel->lcd->printf("%-12s %8.3f", name, value);
    }
}

void ModifyValuesScreen::clicked_menu_entry(uint16_t line)
{
    if (line == 0) {
        this->panel->enter_screen(this->parent);

    } else {
        line--;
        this->selected_item = line;
        this->control_mode = VALUE_CONTROL_MODE;

        const char *name = std::get<0>(menu_items[line]);
        float value = std::get<1>(menu_items[line])();
        float inc= std::get<3>(menu_items[line]);
        this->panel->enter_control_mode(inc, inc / 10);
        this->min_value= std::get<4>(menu_items[line]);
        this->max_value= std::get<5>(menu_items[line]);

        this->panel->set_control_value(value);
        this->panel->lcd->clear();
        this->panel->lcd->setCursor(0, 0);
        this->panel->lcd->printf("%s", name);
        this->panel->lcd->setCursor(0, 2);
        this->panel->lcd->printf("%10.3f", value);
    }
}

// queuing commands needs to be done from main loop
void ModifyValuesScreen::on_main_loop()
{
    // issue command
    if (execute_function == -1) return;

    // execute the setter function for the specified menu item
    std::get<2>(menu_items[execute_function])(new_value);
    execute_function = -1;
}

void ModifyValuesScreen::addMenuItem(const MenuItemType& item)
{
    menu_items.push_back(item);
}

void ModifyValuesScreen::addMenuItem(const char *name, std::function<float()> getter, std::function<void(float)> setter, float inc, float  min, float max)
{
    menu_items.push_back(make_tuple(name, getter, setter, inc, min, max));
}
