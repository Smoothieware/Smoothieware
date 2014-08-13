/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef WATCHSCREEN_H
#define WATCHSCREEN_H

#include "PanelScreen.h"

class WatchScreen : public PanelScreen
{
public:
    WatchScreen();
    ~WatchScreen();
    void on_refresh();
    void on_enter();
    void on_main_loop();
    void display_menu_line(uint16_t line);

private:
    void get_temp_data();
    float get_current_speed();
    void set_speed();
    void get_current_pos(float *cp);
    void get_sd_play_info();
    const char *get_status();
    const char *get_network();

    bool speed_changed;
    bool issue_change_speed;
    bool fan_state;
    int hotendtemp;
    int hotendtarget;
    int bedtemp;
    int bedtarget;
    int current_speed;
    float pos[3];
    unsigned long elapsed_time;
    unsigned int sd_pcnt_played;

    char *ipstr;
};

#endif
