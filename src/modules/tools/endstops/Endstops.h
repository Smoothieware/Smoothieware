/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "Pin.h"

#include <bitset>
#include <array>
#include <map>

class StepperMotor;
class Gcode;
class Pin;

class Endstops : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);

    private:
        bool load_old_config();
        bool load_config();
        void get_global_configs();
        using axis_bitmap_t = std::bitset<6>;
        void home(axis_bitmap_t a);
        void home_xy();
        void back_off_home(axis_bitmap_t axis);
        void move_to_origin(axis_bitmap_t axis);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_idle(void *argument);
        bool debounced_get(Pin *pin);
        void process_home_command(Gcode* gcode);
        void set_homing_offset(Gcode* gcode);
        uint32_t read_endstops(uint32_t dummy);
        void handle_park();

        // global settings
        float saved_position[3]{0}; // save G28 (in grbl mode)
        uint32_t debounce_count;
        uint32_t  debounce_ms;
        axis_bitmap_t axis_to_home;

        float trim_mm[3];

        // per endstop settings
        using endstop_info_t = struct {
            Pin pin;
            struct {
                uint16_t debounce:16;
                char axis:8; // one of XYZABC
                uint8_t axis_index:3;
                bool limit_enable:1;
                bool triggered:1;
            };
        };

        using homing_info_t = struct {
            float homing_position;
            float home_offset;
            float max_travel;
            float retract;
            float fast_rate;
            float slow_rate;
            endstop_info_t *pin_info;

            struct {
                char axis:8; // one of XYZABC
                uint8_t axis_index:3;
                bool home_direction:1; // true min or false max
                bool homed:1;
            };
        };

        // array of endstops
        std::vector<endstop_info_t *> endstops;

        // axis that can be homed, 0,1,2 always there and optionally 3 is A, 4 is B, 5 is C
        std::vector<homing_info_t> homing_axis;

        // Global state
        struct {
            uint32_t homing_order:18;
            volatile char status:3;
            bool is_corexy:1;
            bool is_delta:1;
            bool is_rdelta:1;
            bool is_scara:1;
            bool home_z_first:1;
            bool move_to_origin_after_home:1;
            bool park_after_home:1;
        };
};
