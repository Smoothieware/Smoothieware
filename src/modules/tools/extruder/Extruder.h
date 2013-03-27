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

#define microseconds_per_step_pulse_checksum CHECKSUM("microseconds_per_step_pulse")
#define extruder_module_enable_checksum      CHECKSUM("extruder_module_enable")
#define extruder_steps_per_mm_checksum       CHECKSUM("extruder_steps_per_mm")
#define extruder_acceleration_checksum       CHECKSUM("extruder_acceleration")
#define extruder_step_pin_checksum           CHECKSUM("extruder_step_pin")
#define extruder_dir_pin_checksum            CHECKSUM("extruder_dir_pin")
#define extruder_en_pin_checksum             CHECKSUM("extruder_en_pin")
#define extruder_max_speed_checksum          CHECKSUM("extruder_max_speed")

// default_feed_rate_checksum defined by Robot.h

#define OFF 0
#define SOLO 1
#define FOLLOW 2

class Extruder : public Module{
    public:
        Extruder();
        void     on_module_loaded();
        void     on_config_reload(void* argument);
        void     on_gcode_received(void*);
        void     on_gcode_execute(void* argument);
        void     on_block_begin(void* argument);
        void     on_block_end(void* argument);
        void     on_play(void* argument);
        void     on_pause(void* argument);
        void     on_speed_change(void* argument);
        uint32_t acceleration_tick(uint32_t dummy);
        uint32_t stepper_motor_finished_move(uint32_t dummy);
        Block*   append_empty_block();

        Pin             step_pin;                     // Step pin for the stepper driver
        Pin             dir_pin;                      // Dir pin for the stepper driver
        Pin             en_pin;

        double          target_position;              // End point ( in steps ) for the current move
        double          current_position;             // Current point ( in steps ) for the current move, incremented every time a step is outputed
        int             current_steps;
        Block*          current_block;                // Current block we are stepping, same as Stepper's one
        int             microseconds_per_step_pulse;  // Pulse duration for step pulses
        double          steps_per_millimeter;         // Steps to travel one millimeter
        double          feed_rate;                    //
        double          acceleration;                 //
        double          max_speed;

        int             counter_increment;
        int             step_counter;

        bool            solo_mode;
        double          travel_ratio;
        double          travel_distance;
        bool            absolute_mode;

        bool            debug;
        int debug_count;

        char mode;

        bool paused;

        StepperMotor* stepper_motor;

};

#endif
