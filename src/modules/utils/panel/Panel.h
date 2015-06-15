/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef PANEL_H
#define PANEL_H

#include "Button.h"
#include "Pin.h"
#include "mbed.h"
#include <string>
using std::string;

#define MENU_MODE                  0
#define CONTROL_MODE               1

#define THEPANEL Panel::instance

class LcdBase;
class PanelScreen;
class ModifyValuesScreen;
class SDCard;
class SDFAT;

class Panel : public Module {
    public:
        Panel();
        virtual ~Panel();
        static Panel* instance;

        void on_module_loaded();
        uint32_t button_tick(uint32_t dummy);
        uint32_t encoder_tick(uint32_t dummy);
        void on_idle(void* argument);
        void on_halt(void* argument);
        void on_main_loop(void* argument);
        void on_set_public_data(void* argument);
        void on_second_tick(void* argument);
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
        bool is_suspended() const;
        void set_playing_file(string f);
        const char* get_playing_file() { return playing_file; }

        string getMessage() { return message; }
        bool hasMessage() { return message.size() > 0; }
        bool is_halted() const { return halted; }

        uint16_t get_screen_lines() const { return screen_lines; }

        // public as it is directly accessed by screens... not good
        // TODO pass lcd into ctor of each sub screen
        LcdBase* lcd;
        PanelScreen* custom_screen;

        // as panelscreen accesses private fields in Panel
        friend class PanelScreen;

    private:

        // external SD card
        bool mount_external_sd(bool on);
        Pin sdcd_pin;
        PinName extsd_spi_cs;
        SDCard *sd;
        SDFAT *extmounter;

        // Menu
        int menu_selected_line;
        int menu_start_line;
        int menu_rows;
        int panel_lines;

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

        int idle_time;

        PanelScreen* top_screen;
        PanelScreen* current_screen;

        float jogging_speed_mm_min[3];
        float default_hotend_temperature;
        float default_bed_temperature;

        string message;

        uint16_t screen_lines;
        uint16_t menu_current_line;
        char playing_file[20];
        uint8_t extsd_spi_channel;

        volatile struct {
            bool start_up:1;
            bool menu_changed:1;
            bool control_value_changed:1;
            bool external_sd_enable:1;
            bool halted:1;
            volatile bool counter_changed:1;
            volatile bool click_changed:1;
            volatile bool refresh_flag:1;
            volatile bool do_buttons:1;
            volatile bool do_encoder:1;
            char mode:2;
            char menu_offset:3;
            int encoder_click_resolution:3;
        };
};

#endif
