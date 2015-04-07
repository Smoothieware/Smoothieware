/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "LcdBase.h"
#include "PrepareScreen.h"
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "checksumm.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "TemperatureControlPublicAccess.h"
#include "ModifyValuesScreen.h"
#include "TemperatureControlPool.h"

#include <string>
using namespace std;

PrepareScreen::PrepareScreen()
{
    // Children screens
    if(THEKERNEL->temperature_control_pool->get_controllers().size() > 0) {
        this->extruder_screen = (new ExtruderScreen())->set_parent(this);
    }else{
        this->extruder_screen= nullptr;
    }
}

void PrepareScreen::on_enter()
{
    THEPANEL->enter_menu_mode();
    // if no heaters or extruder then don't show related menu items
    THEPANEL->setup_menu((this->extruder_screen != nullptr) ? 9 : 5);
    this->refresh_menu();
}

void PrepareScreen::on_refresh()
{
    if ( THEPANEL->menu_change() ) {
        this->refresh_menu();
    }
    if ( THEPANEL->click() ) {
        this->clicked_menu_entry(THEPANEL->get_menu_current_line());
    }
}

void PrepareScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("Back"           ); break;
        case 1: THEPANEL->lcd->printf("Home All Axis"  ); break;
        case 2: THEPANEL->lcd->printf("Set Home"       ); break;
        case 3: THEPANEL->lcd->printf("Set Z0"         ); break;
        case 4: THEPANEL->lcd->printf("Motors off"     ); break;
        // these won't be accessed if no heaters or extruders
        case 5: THEPANEL->lcd->printf("Pre Heat"       ); break;
        case 6: THEPANEL->lcd->printf("Cool Down"      ); break;
        case 7: THEPANEL->lcd->printf("Extruder..."    ); break;
        case 8: THEPANEL->lcd->printf("Set Temperature"); break;
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: THEPANEL->enter_screen(this->parent); break;
        case 1: send_command("G28"); break;
        case 2: send_command("G92 X0 Y0 Z0"); break;
        case 3: send_command("G92 Z0"); break;
        case 4: send_command("M84"); break;
        case 5: this->preheat(); break;
        case 6: this->cooldown(); break;
        case 7: THEPANEL->enter_screen(this->extruder_screen); break;
        case 8: setup_temperature_screen(); break;
    }
}

void PrepareScreen::preheat()
{
    float t = THEPANEL->get_default_hotend_temp();
    PublicData::set_value( temperature_control_checksum, hotend_checksum, &t );
    t = THEPANEL->get_default_bed_temp();
    PublicData::set_value( temperature_control_checksum, bed_checksum, &t );
}

void PrepareScreen::cooldown()
{
    float t = 0;
    PublicData::set_value( temperature_control_checksum, hotend_checksum, &t );
    PublicData::set_value( temperature_control_checksum, bed_checksum, &t );
}

static float getTargetTemperature(uint16_t heater_cs)
{
    void *returned_data;
    bool ok = PublicData::get_value( temperature_control_checksum, heater_cs, current_temperature_checksum, &returned_data );

    if (ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
        return temp.target_temperature;
    }

    return 0.0F;
}

void PrepareScreen::setup_temperature_screen()
{
    // setup temperature screen
    auto mvs= new ModifyValuesScreen(true); // delete itself on exit
    mvs->set_parent(this);

    int cnt= 0;
    // returns enabled temperature controllers
    for(auto i : THEKERNEL->temperature_control_pool->get_controllers()) {
        void *returned_data;
        bool ok = PublicData::get_value( temperature_control_checksum, i, current_temperature_checksum, &returned_data );
        if (!ok) continue;

        struct pad_temperature t =  *static_cast<struct pad_temperature *>(returned_data);

        // rename if two of the known types
        const char *name;
        if(t.designator == "T") name= "Hotend";
        else if(t.designator == "B") name= "Bed";
        else name= t.designator.c_str();

        mvs->addMenuItem(name, // menu name
            [i]() -> float { return getTargetTemperature(i); }, // getter
            [i](float t) { PublicData::set_value( temperature_control_checksum, i, &t ); }, // setter
            1.0F, // increment
            0.0F, // Min
            500.0F // Max
        );
        cnt++;
    }

    if(cnt > 0) {
        THEPANEL->enter_screen(mvs);
    }else{
        // no heaters and probably no extruders either
        delete mvs;
    }
}
