/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef EXTURDER_MODULE_H
#define EXTRUDER_MODULE_H

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Block.h"

#define microseconds_per_step_pulse_checksum 42333
#define extruder_module_enable_checksum      6183
#define extruder_steps_per_mm_checksum       58088
#define default_feed_rate_checksum           53183
#define acceleration_checksum                60356

#define OFF 0
#define SOLO 1
#define FOLLOW 2

class Extruder : public Module{
    public:
        Extruder();
        void on_module_loaded();
        void on_config_reload(void* argument);
        void on_gcode_execute(void* argument);
        void on_block_begin(void* argument);
        void on_block_end(void* argument);
        void on_play(void* argument);
        void on_pause(void* argument);
        void set_speed(int steps_per_second);
        uint32_t acceleration_tick(uint32_t dummy);
        uint32_t stepping_tick(uint32_t dummy);
        uint32_t reset_step_pin(uint32_t dummy);

        Pin*            step_pin;                     // Step pin for the stepper driver
        Pin*            dir_pin;                      // Dir pin for the stepper driver
        Pin*            en_pin;

        double          start_position;               // Start point ( in steps ) for the current move
        double          target_position;              // End point ( in steps ) for the current move
        double          current_position;             // Current point ( in steps ) for the current move, incremented every time a step is outputed
        Block*          current_block;                // Current block we are stepping, same as Stepper's one
        int             microseconds_per_step_pulse;  // Pulse duration for step pulses
        double          steps_per_millimeter;         // Steps to travel one millimeter
        double          feed_rate;                    //
        double          acceleration;                 //

        int             counter_increment;
        int             step_counter;

        bool            solo_mode;
        double          travel_ratio;
        double          travel_distance;
        bool            absolute_mode;

        int             direction;

        bool            debug;
        int debug_count;

        char mode;
        bool acceleration_lock;

        bool paused;
};

#endif
