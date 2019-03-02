/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "LcdBase.h"
#include "Panel.h"
#include "PanelScreen.h"
#include "MainMenuScreen.h"
#include "WatchScreen.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Robot.h"
#include "Conveyor.h"
#include "modules/robot/Conveyor.h"
#include "modules/utils/player/PlayerPublicAccess.h"
#include "NetworkPublicAccess.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "StepperMotor.h"
#include "BaseSolution.h"

#ifndef NO_TOOLS_LASER
#include "Laser.h"
#endif

#include <math.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include <algorithm>

using namespace std;

#define laser_checksum CHECKSUM("laser")

WatchScreen::WatchScreen()
{
    speed_changed = false;
    issue_change_speed = false;
    ipstr = nullptr;
    update_counts= 0;
}

WatchScreen::~WatchScreen()
{
    delete[] ipstr;
}

void WatchScreen::on_enter()
{
    THEPANEL->lcd->clear();
    THEPANEL->setup_menu(8);
    get_current_status();
    get_wpos();
    get_sd_play_info();
    this->current_speed = lroundf(get_current_speed());
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
    update_counts++;
    if ( update_counts % 20 == 0 ) {
        get_sd_play_info();
        get_wpos();
        get_current_status();
        if (this->speed_changed) {
            this->issue_change_speed = true; // trigger actual command to change speed
            this->speed_changed = false;
        } else if (!this->issue_change_speed) { // change still queued
            // read it in case it was changed via M220
            this->current_speed = lroundf(get_current_speed());
            THEPANEL->set_control_value(this->current_speed);
            THEPANEL->reset_counter();
        }

        this->refresh_screen(THEPANEL->lcd->hasGraphics() ? true : false); // graphics screens should be cleared
    }
}

void WatchScreen::get_wpos()
{
    // get real time positions
    float mpos[3];
    THEROBOT->get_current_machine_position(mpos);
    Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
    this->wpos[0]= THEROBOT->from_millimeters(std::get<X_AXIS>(wpos));
    this->wpos[1]= THEROBOT->from_millimeters(std::get<Y_AXIS>(wpos));
    this->wpos[2]= THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));
    this->mpos[0]= THEROBOT->from_millimeters(mpos[0]);
    this->mpos[1]= THEROBOT->from_millimeters(mpos[1]);
    this->mpos[2]= THEROBOT->from_millimeters(mpos[2]);

    std::vector<Robot::wcs_t> v= THEROBOT->get_wcs_state();
    char current_wcs= std::get<0>(v[0]);
    this->wcs= wcs2gcode(current_wcs);
}

// queuing gcodes needs to be done from main loop
void WatchScreen::on_main_loop()
{
    if (this->issue_change_speed) {
        this->issue_change_speed = false;
        set_speed();
    }
    PanelScreen::on_main_loop(); // in case any queued commands left
}

// fetch the data we are displaying
void WatchScreen::get_current_status()
{
    // get spindle status
    struct pad_switch s;
    bool ok = PublicData::get_value( switch_checksum, fan_checksum, 0, &s );
    if (ok) {
        this->spindle_state = s.state;
    } else {
        // spindle probably disabled
        this->spindle_state = false;
    }
}

// fetch the data we are displaying
float WatchScreen::get_current_speed()
{
    // in percent
    return 6000.0F / THEROBOT->get_seconds_per_minute();
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
        case 0: THEPANEL->lcd->printf("     WCS      MCS %s", THEROBOT->inch_mode ? "in" : "mm"); break;
        case 1: THEPANEL->lcd->printf("X %8.3f %8.3f", wpos[0], mpos[0]); break;
        case 2: THEPANEL->lcd->printf("Y %8.3f %8.3f", wpos[1], mpos[1]); break;
        case 3: THEPANEL->lcd->printf("Z %8.3f %8.3f", wpos[2], mpos[2]); break;
        case 4: THEPANEL->lcd->printf("%s F%6.1f/%6.1f", this->wcs.c_str(), // display requested feedrate and actual feedrate
            THEROBOT->from_millimeters(THEROBOT->get_feed_rate()),
            THEROBOT->from_millimeters(THEKERNEL->conveyor->get_current_feedrate()*60.0F));
            break;
        case 5: THEPANEL->lcd->printf("%3d%% %2lu:%02lu %3u%% sd", this->current_speed, this->elapsed_time / 60, this->elapsed_time % 60, this->sd_pcnt_played); break;
        case 6:
            if(THEPANEL->has_laser()){
                #ifndef NO_TOOLS_LASER
                Laser *plaser= nullptr;
                if(PublicData::get_value(laser_checksum, (void *)&plaser) && plaser != nullptr) {
                    THEPANEL->lcd->printf("Laser S%1.4f/%1.2f%%", THEROBOT->get_s_value(), plaser->get_current_power());
                }
                #endif
            }
            break;
        case 7: THEPANEL->lcd->printf("%19s", this->get_status()); break;
    }
}

const char *WatchScreen::get_status()
{
    if (THEPANEL->hasMessage())
        return THEPANEL->getMessage().c_str();

    if (THEKERNEL->is_halted())
        return "ALARM";

    if (THEKERNEL->get_feed_hold())
        return "Hold";

    if (THEPANEL->is_suspended())
        return "Suspended";

    if (THEPANEL->is_playing())
        return THEPANEL->get_playing_file();

    if (!THECONVEYOR->is_idle())
        return "Running";

    const char *ip = get_network();
    if (ip == NULL) {
        return "Idle";
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
        if (this->ipstr == nullptr) {
            this->ipstr = new char[n + 1];
        }
        strcpy(this->ipstr, buf);

        return this->ipstr;
    }

    return NULL;
}
