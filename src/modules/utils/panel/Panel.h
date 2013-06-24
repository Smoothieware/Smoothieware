/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef PANEL_H
#define PANEL_H

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "panels/LcdBase.h"
#include "Button.h"
#include "PanelScreen.h"
#include "screens/MainMenuScreen.h"

#define panel_checksum             CHECKSUM("panel")
#define enable_checksum            CHECKSUM("enable")
#define lcd_checksum               CHECKSUM("lcd")
#define i2c_lcd_checksum           CHECKSUM("i2c_lcd")
#define viki_lcd_checksum          CHECKSUM("viki_lcd")

#define jog_x_feedrate_checksum    CHECKSUM("alpha_jog_feedrate")
#define jog_y_feedrate_checksum    CHECKSUM("beta_jog_feedrate")
#define jog_z_feedrate_checksum    CHECKSUM("gamma_jog_feedrate")

#define hotend_temp_checksum CHECKSUM("hotend_temperature")
#define bed_temp_checksum    CHECKSUM("bed_temperature")

#define MENU_MODE                  0
#define CONTROL_MODE               1


class Panel : public Module {
    public:
        Panel();
        virtual ~Panel();

        void on_module_loaded();
        uint32_t button_tick(uint32_t dummy);
        void on_idle(void* argument);
        void on_main_loop(void* argument);
        void enter_screen(PanelScreen* screen);
        void reset_counter();

        // Encoder and buttons
        uint32_t on_up(uint32_t dummy);
        uint32_t on_down(uint32_t dummy);
        uint32_t on_back(uint32_t dummy);
        uint32_t on_click_release(uint32_t dummy);
        uint32_t refresh_tick(uint32_t dummy);
        uint32_t encoder_check(uint32_t dummy);
        bool counter_change();
        bool click();
        int get_encoder_resolution() const { return encoder_click_resolution; }
        
        // Menu
        void enter_menu_mode();
        void setup_menu(uint16_t rows, uint16_t lines);
        void menu_update();
        bool menu_change();
        uint16_t menu_current_line();

        // Control
        bool enter_control_mode(double passed_normal_increment, double passed_pressed_increment);
        void set_control_value(double value);
        double get_control_value();
        bool control_value_change();
        void control_value_update();
        double get_jogging_speed(char axis) { return jogging_speed_mm_min[axis-'X']; }
        double get_default_hotend_temp() { return default_hotend_temperature; }
        double get_default_bed_temp() { return default_bed_temperature; }

        // file playing from sd
        bool is_playing() const;
        void set_playing_file(string f);
        const char* get_playing_file() { return playing_file; }
        
        // public as it is directly accessed by screens... not good
        // TODO pass lcd into ctor of each sub screen
        LcdBase* lcd;

        // as panelscreen accesses private fields in Panel
        friend class PanelScreen;
    
    private:
        // Menu
        char menu_offset;
        int menu_selected_line;
        int menu_start_line;
        int menu_rows;
        int menu_lines;
        bool menu_changed;
        bool control_value_changed;

        // Control
        double normal_increment;
        double pressed_increment;
        int control_normal_counter;
        int control_pressed_counter;
        double control_base_value;
        
        Button up_button;
        Button down_button;
        Button back_button;
        Button click_button;

        int* counter;
        volatile bool counter_changed;
        volatile bool click_changed;
        volatile bool refresh_flag;
        volatile bool do_buttons;
        int idle_time;
        int encoder_click_resolution;
        char mode;

        MainMenuScreen* top_screen;
        PanelScreen* current_screen;

        double jogging_speed_mm_min[3];
        double default_hotend_temperature;
        double default_bed_temperature;
        
        char playing_file[20];
};

#endif
