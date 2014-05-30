/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "PrepareScreen.h"
#include "ExtruderScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "PublicDataRequest.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "PublicData.h"
#include "checksumm.h"
#include "ModifyValuesScreen.h"

#include <string>
using namespace std;

static float getTargetTemperature(uint16_t heater_cs)
{
    void *returned_data;
    bool ok = THEKERNEL->public_data->get_value( temperature_control_checksum, heater_cs, current_temperature_checksum, &returned_data );

    if (ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
        return temp.target_temperature;
    }

    return 0.0F;
}

static void setTargetTemperature(uint16_t heater_cs, float temp)
{
    THEKERNEL->public_data->set_value( temperature_control_checksum, heater_cs, &temp );
}

PrepareScreen::PrepareScreen()
{
    // Children screens
    this->extruder_screen = (new ExtruderScreen()  )->set_parent(this);
    // setup temperature screen
    auto mvs= new ModifyValuesScreen();
    this->temperature_screen= mvs;
    this->temperature_screen->set_parent(this);

    // TODO need to enumerate hotends
    mvs->addMenuItem("Hotend1", // menu name
        []() -> float { return getTargetTemperature(get_checksum("hotend")); }, // getter
        [](float t) { setTargetTemperature(get_checksum("hotend"), t); }, // setter
        1.0F, // increment
        0.0F, // Min
        500.0F // Max
        );
    mvs->addMenuItem("Bed", []() -> float { return getTargetTemperature(get_checksum("bed")); }, [](float t) { setTargetTemperature(get_checksum("bed"), t); }, 1.0F, 0.0F, 500.0F);
}

void PrepareScreen::on_enter()
{
    this->panel->enter_menu_mode();
    this->panel->setup_menu(9);
    this->refresh_menu();
}

void PrepareScreen::on_refresh()
{
    if ( this->panel->menu_change() ) {
        this->refresh_menu();
    }
    if ( this->panel->click() ) {
        this->clicked_menu_entry(this->panel->get_menu_current_line());
    }
}

void PrepareScreen::display_menu_line(uint16_t line)
{
    switch ( line ) {
        case 0: this->panel->lcd->printf("Back"           ); break;
        case 1: this->panel->lcd->printf("Home All Axis"  ); break;
        case 2: this->panel->lcd->printf("Set Home"       ); break;
        case 3: this->panel->lcd->printf("Set Z0"         ); break;
        case 4: this->panel->lcd->printf("Pre Heat"       ); break;
        case 5: this->panel->lcd->printf("Cool Down"      ); break;
        case 6: this->panel->lcd->printf("Extrude"        ); break;
        case 7: this->panel->lcd->printf("Motors off"     ); break;
        case 8: this->panel->lcd->printf("Set Temperature"); break;
    }
}

void PrepareScreen::clicked_menu_entry(uint16_t line)
{
    switch ( line ) {
        case 0: this->panel->enter_screen(this->parent); break;
        case 1: command = "G28"; break;
        case 2: command = "G92 X0 Y0 Z0"; break;
        case 3: command = "G92 Z0"; break;
        case 4: this->preheat(); break;
        case 5: this->cooldown(); break;
        case 6: this->panel->enter_screen(this->extruder_screen); break;
        case 7: command = "M84"; break;
        case 8: this->panel->enter_screen(this->temperature_screen); break;
    }
}

void PrepareScreen::preheat()
{
    float t = panel->get_default_hotend_temp();
    THEKERNEL->public_data->set_value( temperature_control_checksum, hotend_checksum, &t );
    t = panel->get_default_bed_temp();
    THEKERNEL->public_data->set_value( temperature_control_checksum, bed_checksum, &t );
}

void PrepareScreen::cooldown()
{
    float t = 0;
    THEKERNEL->public_data->set_value( temperature_control_checksum, hotend_checksum, &t );
    THEKERNEL->public_data->set_value( temperature_control_checksum, bed_checksum, &t );
}

// queuing commands needs to be done from main loop
void PrepareScreen::on_main_loop()
{
    // change actual axis value
    if (this->command.empty()) return;
    send_command(this->command.c_str());
    this->command.clear();
}

