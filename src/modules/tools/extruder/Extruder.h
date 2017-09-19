/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once

#include "Tool.h"
#include "Pin.h"

#include <tuple>

class StepperMotor;

// NOTE Tool is also a module, no need for multiple inheritance here
class Extruder : public Tool {
    public:
        Extruder(uint16_t config_identifier);
        virtual ~Extruder();

        void     on_module_loaded();
        void     on_gcode_received(void*);

        void select();
        void deselect();
        float get_e_scale(void) const { return volumetric_multiplier * extruder_multiplier; }

    private:
        void config_load();
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        float check_max_speeds(float target, float isecs);
        void save_position();
        void restore_position();

        StepperMotor *stepper_motor;

        float extruder_multiplier;          // flow rate 1.0 == 100%
        float filament_diameter;            // filament diameter
        float volumetric_multiplier;
        float max_volumetric_rate;      // used for calculating volumetric rate in mm³/sec

        // for firmware retract
        float retract_length;               // firmware retract length
        float retract_feedrate;
        float retract_recover_feedrate;
        float retract_recover_length;
        float retract_zlift_length;
        float retract_zlift_feedrate;

        // for saving and restoring extruder position
        std::tuple<float, float, int32_t> saved_position;

        struct {
            uint8_t motor_id:8;
            bool retracted:1;
            bool cancel_zlift_restore:1; // hack to stop a G11 zlift restore from overring an absolute Z setting
            bool selected:1;
            bool saved_selected:1;
            bool g92e0_detected:1;
        };
};
