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
#include "modules/tools/toolsmanager/Tool.h"

#define extruder_module_enable_checksum      CHECKSUM("extruder_module_enable")
#define extruder_steps_per_mm_checksum       CHECKSUM("extruder_steps_per_mm")
#define extruder_acceleration_checksum       CHECKSUM("extruder_acceleration")
#define extruder_step_pin_checksum           CHECKSUM("extruder_step_pin")
#define extruder_dir_pin_checksum            CHECKSUM("extruder_dir_pin")
#define extruder_en_pin_checksum             CHECKSUM("extruder_en_pin")
#define extruder_max_speed_checksum          CHECKSUM("extruder_max_speed")

#define extruder_checksum                    CHECKSUM("extruder")

#define steps_per_mm_checksum                CHECKSUM("steps_per_mm")
#define acceleration_checksum                CHECKSUM("acceleration")
#define step_pin_checksum                    CHECKSUM("step_pin")
#define dir_pin_checksum                     CHECKSUM("dir_pin")
#define en_pin_checksum                      CHECKSUM("en_pin")
#define max_speed_checksum                   CHECKSUM("max_speed")



// default_feed_rate_checksum defined by Robot.h

#define OFF 0
#define SOLO 1
#define FOLLOW 2

class Extruder : public Module, public Tool {
    public:
        Extruder(uint16_t config_identifier);
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

        double          target_position;              // End point ( in mm ) for the current move
        double          current_position;             // Current point ( in mm ) for the current move, incremented every time a move is executed
        double          unstepped_distance;           // overflow buffer for requested moves that are less than 1 step
        Block*          current_block;                // Current block we are stepping, same as Stepper's one
        double          steps_per_millimeter;         // Steps to travel one millimeter
        double          feed_rate;                    //
        double          acceleration;                 //
        double          max_speed;

        double          travel_ratio;
        double          travel_distance;
        bool            absolute_mode;                // absolute/relative coordinate mode switch

        char mode;                                    // extruder motion mode,  OFF, SOLO, or FOLLOW

        bool paused;
        bool single_config;

        uint16_t identifier;

        StepperMotor* stepper_motor;

};

#endif
