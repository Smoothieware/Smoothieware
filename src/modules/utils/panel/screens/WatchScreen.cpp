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

WatchScreen::WatchScreen(){}

void WatchScreen::on_enter(){
    this->panel->lcd->clear();
    this->panel->setup_menu(4, 4);
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
        // change actual speed
        set_current_speed();
        this->refresh_screen(false);
    }
    
    // Update Only every 20 refreshes, 1 a second
    static int update_counts = 0;
    update_counts++;
    if( update_counts % 20 == 0 ){
        get_sd_play_info();
        get_current_pos(this->pos);
        get_temp_data();
        this->current_speed= get_current_speed();
        this->refresh_screen(false);
    }
}

// fetch the data we are displaying
void WatchScreen::get_temp_data() {
    void *returned_data;
    bool ok;

    ok= THEKERNEL->public_data->get_value( temperature_control_checksum, bed_checksum, current_temperature_checksum, &returned_data );
    if(ok) {
        struct pad_temperature temp=  *static_cast<struct pad_temperature*>(returned_data);
        this->bedtemp= round(temp.current_temperature);
        this->bedtarget= round(temp.target_temperature);
        //this->bedpwm= temp.pwm;
    }

    ok= THEKERNEL->public_data->get_value( temperature_control_checksum, hotend_checksum, current_temperature_checksum, &returned_data );
    if(ok) {
        struct pad_temperature temp=  *static_cast<struct pad_temperature*>(returned_data);
        this->hotendtemp= round(temp.current_temperature);
        this->hotendtarget= round(temp.target_temperature);
        //this->hotendpwm= temp.pwm;
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

void WatchScreen::set_current_speed() {
    bool ok= THEKERNEL->public_data->set_value( robot_checksum, speed_override_percent_checksum, &this->current_speed );
    if(!ok) this->current_speed= 0;
        
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
        case 3: this->panel->lcd->printf("%19s", panel->is_playing() ? panel->get_playing_file() : "Smoothie ready"); break;
    }
}
