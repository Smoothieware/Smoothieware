/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "PanelScreen.h"

class DirectJogScreen : public PanelScreen
{
public:
    DirectJogScreen();
    void on_refresh();
    void on_enter();
    void on_exit();
    void on_main_loop() {}

    void display_menu_line(uint16_t line);
    int idle_timeout_secs() { return 120; }

private:
    void clicked_menu_entry(uint16_t line);
    void display_axis_line(uint8_t axis);
    void enter_axis_control(uint8_t axis);
    void enter_menu_control();
    void tick(int change);
    void get_actuator_pos();

    float pos[3];

    enum MODE_T { JOG, AXIS_SELECT, MULTIPLIER };
    MODE_T mode;
    uint8_t axis;
    uint8_t multiplier;
    bool pos_changed;
};
