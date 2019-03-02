/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "CustomScreen.h"
#include "libs/Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "LcdBase.h"
#include "Panel.h"
#include "ConfigValue.h"

#include <string.h>

#include <algorithm>

#define enable_checksum      CHECKSUM("enable")
#define custom_menu_checksum CHECKSUM("custom_menu")
#define name_checksum        CHECKSUM("name")
#define command_checksum     CHECKSUM("command")

using namespace std;

CustomScreen::CustomScreen()
{
    //printf("Setting up CustomScreen\n");
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, custom_menu_checksum );

    // load the custom menu items
    for ( unsigned int i = 0; i < modules.size(); i++ ) {
        if (THEKERNEL->config->value(custom_menu_checksum, modules[i], enable_checksum )->as_bool()) {
            // Get Menu entry name
            string name = THEKERNEL->config->value(custom_menu_checksum, modules[i], name_checksum )->as_string();
            std::replace( name.begin(), name.end(), '_', ' '); // replace _ with space

            // Get Command
            string command = THEKERNEL->config->value(custom_menu_checksum, modules[i], command_checksum )->as_string();
            std::replace( command.begin(), command.end(), '_', ' '); // replace _ with space
            std::replace( command.begin(), command.end(), '|', '\n'); // replace | with \n for multiple commands

            // put in menu item list
            menu_items.push_back(make_tuple(strdup(name.c_str()), strdup(command.c_str())));
            //printf("added menu %s, command %s\n", name.c_str(), command.c_str());
        }
    }
}

void CustomScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    THEPANEL->setup_menu(menu_items.size() + 1);
    this->refresh_menu();
}

void CustomScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void CustomScreen::display_menu_line(uint16_t line)
{
    if (line == 0) {
        THEPANEL->lcd->printf("Back");
    } else {
        THEPANEL->lcd->printf(std::get<0>(menu_items[line-1]));
    }
}

void CustomScreen::clicked_menu_entry(uint16_t line)
{
    if (line == 0) {
        THEPANEL->enter_screen(this->parent);
    } else {
        send_command(std::get<1>(menu_items[line-1])); // will be done in main loop
    }
}
