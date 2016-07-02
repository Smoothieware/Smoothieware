/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/Pin.h"

#include <bitset>
#include <array>

class StepperMotor;
class Gcode;

class Endstops : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);

    private:
        void load_config();
        void home(std::bitset<3> a);
        void home_xy();
        void back_off_home(std::bitset<3> axis);
        void move_to_origin(std::bitset<3> axis);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_idle(void *argument);
        bool debounced_get(int pin);
        void process_home_command(Gcode* gcode);
        void set_homing_offset(Gcode* gcode);
        uint32_t read_endstops(uint32_t dummy);

        float homing_position[3];
        float home_offset[3];
        float saved_position[3]{0}; // save G28 (in grbl mode)
        float alpha_max, beta_max, gamma_max;

        uint32_t debounce_count;
        uint32_t  debounce_ms;
        float  retract_mm[3];
        float  trim_mm[3];
        float  fast_rates[3];
        float  slow_rates[3];
        float  retract_rate;
        Pin    pins[6];
        std::array<uint16_t, 3> debounce;

        std::bitset<3> home_direction;
        std::bitset<3> limit_enable;
        std::bitset<3> axis_to_home;

        struct {
            uint8_t homing_order:6;
            uint8_t bounce_cnt:4;
            volatile char status:3;
            bool is_corexy:1;
            bool is_delta:1;
            bool is_rdelta:1;
            bool is_scara:1;
            bool home_z_first:1;
            bool move_to_origin_after_home:1;
        };
};
