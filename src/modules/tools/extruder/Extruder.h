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
        virtual ~Extruder();

        void     on_module_loaded();
        void     on_config_reload(void* argument);
        void     on_gcode_received(void*);
        void     on_gcode_execute(void* argument);
        void     on_block_begin(void* argument);
        void     on_block_end(void* argument);
        void     on_play(void* argument);
        void     on_pause(void* argument);
        void     on_halt(void* argument);
        void     on_speed_change(void* argument);
        void     acceleration_tick(void);
        uint32_t stepper_motor_finished_move(uint32_t dummy);
        Block*   append_empty_block();

    private:
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        uint32_t rate_increase() const;
        float check_max_speeds(float target, float isecs);

        StepperMotor*  stepper_motor;
        Pin            step_pin;                     // Step pin for the stepper driver
        Pin            dir_pin;                      // Dir pin for the stepper driver
        Pin            en_pin;
        float          target_position;              // End point ( in mm ) for the current move
        float          unstepped_distance;           // overflow buffer for requested moves that are less than 1 step
        Block*         current_block;                // Current block we are stepping, same as Stepper's one

        // kept together so they can be passed as public data
        struct {
            float steps_per_millimeter;         // Steps to travel one millimeter
            float filament_diameter;            // filament diameter
            float extruder_multiplier;          // flow rate 1.0 == 100%
            float acceleration;                 // extruder accleration SOLO setting
            float retract_length;               // firmware retract length
            float current_position;             // Current point ( in mm ) for the current move, incremented every time a move is executed
        };

        float saved_current_position;
        float volumetric_multiplier;
        float feed_rate;                // default rate mm/sec for SOLO moves only
        float milestone_last_position;  // used for calculating volumemetric rate, last position in mm³
        float max_volumetric_rate;      // used for calculating volumetric rate in mm³/sec

        float travel_ratio;
        float travel_distance;

        // for firmware retract
        float retract_feedrate;
        float retract_recover_feedrate;
        float retract_recover_length;
        float retract_zlift_length;
        float retract_zlift_feedrate;

        struct {
            char mode:3;        // extruder motion mode,  OFF, SOLO, or FOLLOW
            bool absolute_mode:1; // absolute/relative coordinate mode switch
            bool saved_absolute_mode:1;
            bool paused:1;
            bool single_config:1;
            bool retracted:1;
            bool cancel_zlift_restore:1; // hack to stop a G11 zlift restore from overring an absolute Z setting
            bool milestone_absolute_mode:1;
        };


};

#endif
