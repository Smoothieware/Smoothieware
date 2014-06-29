/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PANEL_H
#define PANEL_H

#include "Button.h"
#include <string>
using std::string;

#define MENU_MODE                  0
#define CONTROL_MODE               1

#define THEPANEL Panel::instance

class LcdBase;
class PanelScreen;

class Panel : public Module {
    public:
        Panel();
        virtual ~Panel();
        static Panel* instance;

        void on_module_loaded();
        uint32_t button_tick(uint32_t dummy);
        uint32_t encoder_tick(uint32_t dummy);
        void on_idle(void* argument);
        void on_main_loop(void* argument);
        void on_gcode_received(void* argument);
        void enter_screen(PanelScreen* screen);
        void reset_counter();

        // Encoder and buttons
        uint32_t on_up(uint32_t dummy);
        uint32_t on_down(uint32_t dummy);
        uint32_t on_back(uint32_t dummy);
        uint32_t on_select(uint32_t dummy);
        uint32_t on_pause(uint32_t dummy);
        uint32_t refresh_tick(uint32_t dummy);
        uint32_t encoder_check(uint32_t dummy);
        bool counter_change();
        bool click();
        int get_encoder_resolution() const { return encoder_click_resolution; }

        // Menu
        void enter_menu_mode(bool force= false);
        void setup_menu(uint16_t rows, uint16_t lines);
        void setup_menu(uint16_t rows);
        void menu_update();
        bool menu_change();
        uint16_t max_screen_lines() { return screen_lines; }
        uint16_t get_menu_current_line() { return menu_current_line; }

        // Control
        bool enter_control_mode(float passed_normal_increment, float passed_pressed_increment);
        void set_control_value(float value);
        float get_control_value();
        bool control_value_change();
        void control_value_update();
        float get_jogging_speed(char axis) { return jogging_speed_mm_min[axis-'X']; }
        float get_default_hotend_temp() { return default_hotend_temperature; }
        float get_default_bed_temp() { return default_bed_temperature; }

        // file playing from sd
        bool is_playing() const;
        void set_playing_file(string f);
        const char* get_playing_file() { return playing_file; }

        string getMessage() { return message; }
        bool hasMessage() { return message.size() > 0; }

        // public as it is directly accessed by screens... not good
        // TODO pass lcd into ctor of each sub screen
        LcdBase* lcd;
        PanelScreen* custom_screen;
        PanelScreen* temperature_screen;

        // as panelscreen accesses private fields in Panel
        friend class PanelScreen;

    private:
        void setup_temperature_screen();

        // Menu
        char menu_offset;
        int menu_selected_line;
        int menu_start_line;
        int menu_rows;
        int panel_lines;
        bool menu_changed;
        bool control_value_changed;
        uint16_t menu_current_line;

        // Control
        float normal_increment;
        int control_normal_counter;
        float control_base_value;

        Button up_button;
        Button down_button;
        Button back_button;
        Button click_button;
        Button pause_button;

        int* counter;
        volatile bool counter_changed;
        volatile bool click_changed;
        volatile bool refresh_flag;
        volatile bool do_buttons;
        volatile bool do_encoder;
        bool paused;
        int idle_time;
        bool start_up;
        int encoder_click_resolution;
        char mode;
        uint16_t screen_lines;

        PanelScreen* top_screen;
        PanelScreen* current_screen;

        float jogging_speed_mm_min[3];
        float default_hotend_temperature;
        float default_bed_temperature;

        char playing_file[20];
        string message;
};

#endif
