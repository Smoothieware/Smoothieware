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
#include "modules/utils/player/PlayerPublicAccess.h"

#include <string>
using namespace std;
static const uint8_t icons[] = { // 115x19 - 3 bytes each: he1, he2, he3, bed, fan
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xFF,0xE0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0xE0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x63,0x0C,0x60,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x47,0x0E,0x20,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4F,0x0F,0x20,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5F,0x0F,0xA0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5E,0x07,0xA0,
0x7F,0x80,0x00,0x3F,0xC0,0x00,0x3F,0xC0,0x00,0x41,0x04,0x00,0x40,0x60,0x20,
0xFB,0xC0,0x00,0x79,0xE0,0x00,0x79,0xE0,0x00,0x20,0x82,0x00,0x40,0xF0,0x20,
0xF3,0xC0,0x00,0x76,0xE0,0x00,0x76,0xE0,0x00,0x20,0x82,0x00,0x40,0xF0,0x20,
0xEB,0xC0,0x00,0x7E,0xE0,0x00,0x7E,0xE0,0x00,0x41,0x04,0x00,0x40,0x60,0x20,
0x7B,0x80,0x00,0x3D,0xC0,0x00,0x39,0xC0,0x00,0x82,0x08,0x00,0x5E,0x07,0xA0,
0x7B,0x80,0x00,0x3B,0xC0,0x00,0x3E,0xC0,0x01,0x04,0x10,0x00,0x5F,0x0F,0xA0,
0xFB,0xC0,0x00,0x77,0xE0,0x00,0x76,0xE0,0x01,0x04,0x10,0x00,0x4F,0x0F,0x20,
0xFB,0xC0,0x00,0x70,0xE0,0x00,0x79,0xE0,0x00,0x82,0x08,0x00,0x47,0x0E,0x20,
0xFF,0xC0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x41,0x04,0x00,0x63,0x0C,0x60,
0x3F,0x00,0x00,0x1F,0x80,0x00,0x1F,0x80,0x00,0x00,0x00,0x00,0x70,0x00,0xE0,
0x1E,0x00,0x00,0x0F,0x00,0x00,0x0F,0x00,0x01,0xFF,0xFF,0x80,0x7F,0xFF,0xE0,
0x0C,0x00,0x00,0x06,0x00,0x00,0x06,0x00,0x01,0xFF,0xFF,0x80,0x00,0x00,0x00
};



WatchScreen::WatchScreen(){
    speed_changed= false;
}

void WatchScreen::on_enter(){
    this->panel->lcd->clear();
    this->panel->setup_menu(4);
    get_temp_data();
    get_current_pos(this->pos);
    get_sd_play_info();
    this->current_speed= get_current_speed();
    this->refresh_screen(false);
    this->panel->enter_control_mode(1,0.5);
    this->panel->set_control_value(this->current_speed);
}

void WatchScreen::on_refresh(){
    // Exit if the button is clicked
    if( this->panel->click() ){
        this->panel->enter_screen(this->parent);
        return;
    }

    // see if speed is being changed
    if(this->panel->control_value_change()) {
        this->current_speed= this->panel->get_control_value();
        if(this->current_speed < 10.0) {
            this->current_speed= 10.0;
            this->panel->set_control_value(this->current_speed);
            this->panel->reset_counter();
        }else{
            // change actual speed
            this->speed_changed= true; // flag main loop to issue g code
            this->refresh_screen(false);
        }
    }

    // Update Only every 20 refreshes, 1 a second
    static int update_counts = 0;
    update_counts++;
    if( update_counts % 20 == 0 ){
        get_sd_play_info();
        get_current_pos(this->pos);
        get_temp_data();
        this->current_speed= get_current_speed();
        this->panel->set_control_value(this->current_speed); // in case it was changed via M220
        this->panel->reset_counter();

        this->refresh_screen(false);

        // for LCDs with leds set them according to heater status
        // TODO should be enabled and disabled and settable from config
        this->panel->lcd->setLed(LED_BED_ON, this->bedtarget > 0);
        this->panel->lcd->setLed(LED_HOTEND_ON, this->hotendtarget > 0);
        //this->panel->lcd->setLed(LED_FAN_ON, this->fanon);

        if(this->panel->lcd->hasGraphics()) {
            // display the graphical icons below the status are
            //this->panel->lcd->bltGlyph(0, 34, 115, 19, icons);
            // for (int i = 0; i < 5; ++i) {
            //     this->panel->lcd->bltGlyph(i*24, 38, 23, 19, icons, 15, i*24, 0);
            // }
            if(this->hotendtarget > 0)
                this->panel->lcd->bltGlyph(8, 38, 20, 19, icons, 15, 0, 0);

            if(this->bedtarget > 0)
                this->panel->lcd->bltGlyph(32, 38, 23, 19, icons, 15, 64, 0);

            // fan appears always on for now
            this->panel->lcd->bltGlyph(96, 38, 23, 19, icons, 15, 96, 0);
        }
    }
}

// queuing gcodes needs to be done from main loop
void WatchScreen::on_main_loop() {
    if(!this->speed_changed) return;
    this->speed_changed= false;
    set_speed();
}

// fetch the data we are displaying
void WatchScreen::get_temp_data() {
    void *returned_data;
    bool ok;

    ok= THEKERNEL->public_data->get_value( temperature_control_checksum, bed_checksum, current_temperature_checksum, &returned_data );
    if(ok) {
        struct pad_temperature temp=  *static_cast<struct pad_temperature*>(returned_data);
        this->bedtemp= round(temp.current_temperature);
        if(this->bedtemp > 100000) this->bedtemp = -2;
        this->bedtarget= round(temp.target_temperature);
        //this->bedpwm= temp.pwm;
    }else{
        // temp probably disabled
        this->bedtemp= -1;
        this->bedtarget= -1;
    }

    ok= THEKERNEL->public_data->get_value( temperature_control_checksum, hotend_checksum, current_temperature_checksum, &returned_data );
    if(ok) {
        struct pad_temperature temp=  *static_cast<struct pad_temperature*>(returned_data);
        this->hotendtemp= round(temp.current_temperature);
        if(this->hotendtemp > 100000) this->hotendtemp = -2;
        this->hotendtarget= round(temp.target_temperature);
        //this->hotendpwm= temp.pwm;
    }else{
        // temp probably disabled
        this->hotendtemp= -1;
        this->hotendtarget= -1;
    }
}

// fetch the data we are displaying
double WatchScreen::get_current_speed() {
    void *returned_data;

    bool ok= THEKERNEL->public_data->get_value( robot_checksum, speed_override_percent_checksum, &returned_data );
    if(ok) {
        double cs= *static_cast<double *>(returned_data);
        return cs;
    }
    return 0.0;
}

void WatchScreen::get_current_pos(double *cp){
    void *returned_data;

    bool ok= THEKERNEL->public_data->get_value( robot_checksum, current_position_checksum, &returned_data );
    if(ok) {
        double *p= static_cast<double *>(returned_data);
        cp[0]= p[0];
        cp[1]= p[1];
        cp[2]= p[2];
    }
}

void WatchScreen::get_sd_play_info(){
    void *returned_data;
    bool ok= THEKERNEL->public_data->get_value( player_checksum, get_progress_checksum, &returned_data );
    if(ok) {
        struct pad_progress p=  *static_cast<struct pad_progress*>(returned_data);
        this->elapsed_time= p.elapsed_secs;
        this->sd_pcnt_played= p.percent_complete;
    }else{
        this->elapsed_time= 0;
        this->sd_pcnt_played= 0;
    }
}

void WatchScreen::display_menu_line(uint16_t line){
    // in menu mode
    switch( line ){
        case 0: this->panel->lcd->printf("H%03d/%03dc B%03d/%03dc", this->hotendtemp, this->hotendtarget, this->bedtemp, this->bedtarget); break;
        case 1: this->panel->lcd->printf("X%4d Y%4d Z%7.2f", (int)round(this->pos[0]), (int)round(this->pos[1]), this->pos[2]); break;
        case 2: this->panel->lcd->printf("%3d%% %2d:%02d %3d%% sd", (int)round(this->current_speed), this->elapsed_time/60, this->elapsed_time%60, this->sd_pcnt_played); break;
        case 3: this->panel->lcd->printf("%19s", this->get_status()); break;
    }
}

const char* WatchScreen::get_status(){
    if(THEKERNEL->pauser->paused())
        return "Paused";

    if(panel->is_playing())
        return panel->get_playing_file();

    return "Smoothie ready";
}

void WatchScreen::set_speed(){
    // change pos by issuing a M220 Snnn
    char buf[32];
    int n= snprintf(buf, sizeof(buf), "M220 S%f", this->current_speed);
    string g(buf, n);
    send_gcode(g);
}
