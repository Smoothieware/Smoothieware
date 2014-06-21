/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef EXTURDER_MODULE_H
#define EXTRUDER_MODULE_H

#include "Tool.h"
#include "Pin.h"

class StepperMotor;
class Block;

// NOTE Tool is also a module, no need for multiple inheritance here
class Extruder : public Tool {
    public:
        Extruder(uint16_t config_identifier, bool single= false);
        virtual ~Extruder() {}

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

    private:
        void on_get_public_data(void* argument);
        void update_steps_per_millimeter();

        Pin             step_pin;                     // Step pin for the stepper driver
        Pin             dir_pin;                      // Dir pin for the stepper driver
        Pin             en_pin;

        float          target_position;              // End point ( in mm ) for the current move
        float          current_position;             // Current point ( in mm ) for the current move, incremented every time a move is executed
        float          unstepped_distance;           // overflow buffer for requested moves that are less than 1 step
        Block*         current_block;                // Current block we are stepping, same as Stepper's one

        float          steps_per_millimeter;         // Steps to travel one millimeter

        // kept together so they can be passed as public data
        struct {
            float          steps_per_millimeter_setting; // original steps to travel one millimeter as set in config, saved while in volumetric mode
            float          filament_diameter;            // filament diameter
        };

        float          feed_rate;                    //
        float          acceleration;                 //
        float          max_speed;

        float          travel_ratio;
        float          travel_distance;

        char mode;        // extruder motion mode,  OFF, SOLO, or FOLLOW
        struct {
            bool absolute_mode:1; // absolute/relative coordinate mode switch
            bool paused:1;
            bool single_config:1;
        };

        StepperMotor* stepper_motor;

};

#endif
