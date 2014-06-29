/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "MainMenuScreen.h"
#include "WatchScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPublicAccess.h"
#include "modules/robot/RobotPublicAccess.h"
#include "modules/robot/Conveyor.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "NetworkPublicAccess.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "Pauser.h"

#include <math.h>
#include <string.h>
#include <string>

using namespace std;
static const uint8_t icons[] = { // 115x19 - 3 bytes each: he1, he2, he3, bed, fan
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x0C, 0x60,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0E, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x0F, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x0F, 0xA0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x07, 0xA0,
    0x7F, 0x80, 0x00, 0x3F, 0xC0, 0x00, 0x3F, 0xC0, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0xFB, 0xC0, 0x00, 0x79, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xF3, 0xC0, 0x00, 0x76, 0xE0, 0x00, 0x76, 0xE0, 0x00, 0x20, 0x82, 0x00, 0x40, 0xF0, 0x20,
    0xEB, 0xC0, 0x00, 0x7E, 0xE0, 0x00, 0x7E, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x40, 0x60, 0x20,
    0x7B, 0x80, 0x00, 0x3D, 0xC0, 0x00, 0x39, 0xC0, 0x00, 0x82, 0x08, 0x00, 0x5E, 0x07, 0xA0,
    0x7B, 0x80, 0x00, 0x3B, 0xC0, 0x00, 0x3E, 0xC0, 0x01, 0x04, 0x10, 0x00, 0x5F, 0x0F, 0xA0,
    0xFB, 0xC0, 0x00, 0x77, 0xE0, 0x00, 0x76, 0xE0, 0x01, 0x04, 0x10, 0x00, 0x4F, 0x0F, 0x20,
    0xFB, 0xC0, 0x00, 0x70, 0xE0, 0x00, 0x79, 0xE0, 0x00, 0x82, 0x08, 0x00, 0x47, 0x0E, 0x20,
    0xFF, 0xC0, 0x00, 0x7F, 0xE0, 0x00, 0x7F, 0xE0, 0x00, 0x41, 0x04, 0x00, 0x63, 0x0C, 0x60,
    0x3F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xE0,
    0x1E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x0F, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x7F, 0xFF, 0xE0,
    0x0C, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00
};



WatchScreen::WatchScreen()
{
    speed_changed = false;
    issue_change_speed = false;
    ipstr = NULL;
}

void WatchScreen::on_enter()
{
    THEPANEL->lcd->clear();
    THEPANEL->setup_menu(4);
    get_temp_data();
    get_current_pos(this->pos);
    get_sd_play_info();
    this->current_speed = lround(get_current_speed());
    this->refresh_screen(false);
    THEPANEL->enter_control_mode(1, 0.5);
    THEPANEL->set_control_value(this->current_speed);
}

void WatchScreen::on_refresh()
{
    // Exit if the button is clicked
    if ( THEPANEL->click() ) {
        THEPANEL->enter_screen(this->parent);
        return;
    }

    // see if speed is being changed
    if (THEPANEL->control_value_change()) {
        this->current_speed = THEPANEL->get_control_value();
        if (this->current_speed < 10) {
            this->current_speed = 10;
            THEPANEL->set_control_value(this->current_speed);
            THEPANEL->reset_counter();
        } else {
            // flag the update to change the speed, we don't want to issue hundreds of M220s
            // but we do want to display the change we are going to make
            this->speed_changed = true; // flag indicating speed changed
            this->refresh_screen(false);
        }
    }

    // Update Only every 20 refreshes, 1 a second
    static int update_counts = 0;
    update_counts++;
    if ( update_counts % 20 == 0 ) {
        get_sd_play_info();
        get_current_pos(this->pos);
        get_temp_data();
        if (this->speed_changed) {
            this->issue_change_speed = true; // trigger actual command to change speed
            this->speed_changed = false;
        } else if (!this->issue_change_speed) { // change still queued
            // read it in case it was changed via M220
            this->current_speed = lround(get_current_speed());
            THEPANEL->set_control_value(this->current_speed);
            THEPANEL->reset_counter();
        }

        this->refresh_screen(THEPANEL->lcd->hasGraphics() ? true : false); // graphics screens should be cleared

        // for LCDs with leds set them according to heater status
        // TODO should be enabled and disabled and settable from config
        THEPANEL->lcd->setLed(LED_BED_ON, this->bedtarget > 0);
        THEPANEL->lcd->setLed(LED_HOTEND_ON, this->hotendtarget > 0);
        THEPANEL->lcd->setLed(LED_FAN_ON, this->fan_state);

        if (THEPANEL->lcd->hasGraphics()) {
            // display the graphical icons below the status are
            //THEPANEL->lcd->bltGlyph(0, 34, 115, 19, icons);
            // for (int i = 0; i < 5; ++i) {
            //     THEPANEL->lcd->bltGlyph(i*24, 38, 23, 19, icons, 15, i*24, 0);
            // }
            if (this->hotendtarget > 0)
                THEPANEL->lcd->bltGlyph(8, 38, 20, 19, icons, 15, 0, 0);

            if (this->bedtarget > 0)
                THEPANEL->lcd->bltGlyph(32, 38, 23, 19, icons, 15, 64, 0);

            if(this->fan_state)
                THEPANEL->lcd->bltGlyph(96, 38, 23, 19, icons, 15, 96, 0);
        }
    }
}

// queuing gcodes needs to be done from main loop
void WatchScreen::on_main_loop()
{
    if (this->issue_change_speed) {
        this->issue_change_speed = false;
        set_speed();
    }
}

// fetch the data we are displaying
void WatchScreen::get_temp_data()
{
    void *returned_data;
    bool ok;

    ok = PublicData::get_value( temperature_control_checksum, bed_checksum, current_temperature_checksum, &returned_data );
    if (ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
        this->bedtemp = round(temp.current_temperature);
        if (this->bedtemp > 100000) this->bedtemp = -2;
        this->bedtarget = round(temp.target_temperature);
        //this->bedpwm= temp.pwm;
    } else {
        // temp probably disabled
        this->bedtemp = -1;
        this->bedtarget = -1;
    }


    ok = PublicData::get_value( temperature_control_checksum, hotend_checksum, current_temperature_checksum, &returned_data );
    if (ok) {
        struct pad_temperature temp =  *static_cast<struct pad_temperature *>(returned_data);
        this->hotendtemp = round(temp.current_temperature);
        if (this->hotendtemp > 100000) this->hotendtemp = -2;
        this->hotendtarget = round(temp.target_temperature);
        //this->hotendpwm= temp.pwm;
    } else {
        // temp probably disabled
        this->hotendtemp = -1;
        this->hotendtarget = -1;
    }

    // get fan status
    ok = PublicData::get_value( switch_checksum, fan_checksum, 0, &returned_data );
    if (ok) {
        struct pad_switch s = *static_cast<struct pad_switch *>(returned_data);
        this->fan_state = s.state;
    } else {
        // fan probably disabled
        this->fan_state = false;
    }
}

// fetch the data we are displaying
float WatchScreen::get_current_speed()
{
    void *returned_data;

    bool ok = PublicData::get_value( robot_checksum, speed_override_percent_checksum, &returned_data );
    if (ok) {
        float cs = *static_cast<float *>(returned_data);
        return cs;
    }
    return 0.0;
}

void WatchScreen::get_current_pos(float *cp)
{
    void *returned_data;

    bool ok = PublicData::get_value( robot_checksum, current_position_checksum, &returned_data );
    if (ok) {
        float *p = static_cast<float *>(returned_data);
        cp[0] = p[0];
        cp[1] = p[1];
        cp[2] = p[2];
    }
}

void WatchScreen::get_sd_play_info()
{
    void *returned_data;
    bool ok = PublicData::get_value( player_checksum, get_progress_checksum, &returned_data );
    if (ok) {
        struct pad_progress p =  *static_cast<struct pad_progress *>(returned_data);
        this->elapsed_time = p.elapsed_secs;
        this->sd_pcnt_played = p.percent_complete;
        THEPANEL->set_playing_file(p.filename);

    } else {
        this->elapsed_time = 0;
        this->sd_pcnt_played = 0;
    }
}

void WatchScreen::display_menu_line(uint16_t line)
{
    // in menu mode
    switch ( line ) {
        case 0: THEPANEL->lcd->printf("H%03d/%03dc B%03d/%03dc", this->hotendtemp, this->hotendtarget, this->bedtemp, this->bedtarget); break;
        case 1: THEPANEL->lcd->printf("X%4d Y%4d Z%7.2f", (int)round(this->pos[0]), (int)round(this->pos[1]), this->pos[2]); break;
        case 2: THEPANEL->lcd->printf("%3d%% %2lu:%02lu %3u%% sd", this->current_speed, this->elapsed_time / 60, this->elapsed_time % 60, this->sd_pcnt_played); break;
        case 3: THEPANEL->lcd->printf("%19s", this->get_status()); break;
    }
}

const char *WatchScreen::get_status()
{
    if (THEPANEL->hasMessage()) {
        return THEPANEL->getMessage().c_str();
    }

    if (THEKERNEL->pauser->paused())
        return "Paused";

    if (THEPANEL->is_playing())
        return THEPANEL->get_playing_file();

    if (!THEKERNEL->conveyor->is_queue_empty()) {
        return "Printing";
    }

    const char *ip = get_network();
    if (ip == NULL) {
        return "Smoothie ready";
    } else {
        return ip;
    }
}

void WatchScreen::set_speed()
{
    send_gcode("M220", 'S', this->current_speed);
}

const char *WatchScreen::get_network()
{
    void *returned_data;

    bool ok = PublicData::get_value( network_checksum, get_ip_checksum, &returned_data );
    if (ok) {
        uint8_t *ipaddr = (uint8_t *)returned_data;
        char buf[20];
        int n = snprintf(buf, sizeof(buf), "IP %d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
        buf[n] = 0;
        if (this->ipstr == NULL) {
            this->ipstr = (char *)malloc(n + 1);
        }
        strcpy(this->ipstr, buf);

        return this->ipstr;
    }

    return NULL;
}
