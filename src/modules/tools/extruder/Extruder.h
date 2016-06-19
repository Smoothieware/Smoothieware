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
        void     on_gcode_received(void*);
        void     on_halt(void* argument);

        void select();
        void deselect();

    private:
        void config_load();
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        uint32_t rate_increase() const;
        float check_max_speeds(float target, float isecs);

        uint8_t        motor_id;

        // kept together so they can be passed as public data
        struct {
            float filament_diameter;            // filament diameter
            float extruder_multiplier;          // flow rate 1.0 == 100%
            float retract_length;               // firmware retract length
        };

        float saved_current_position;
        float volumetric_multiplier;
        float max_volumetric_rate;      // used for calculating volumetric rate in mm³/sec

        // for firmware retract
        float retract_feedrate;
        float retract_recover_feedrate;
        float retract_recover_length;
        float retract_zlift_length;
        float retract_zlift_feedrate;

        struct {
            bool retracted:1;
            bool cancel_zlift_restore:1; // hack to stop a G11 zlift restore from overring an absolute Z setting
            bool selected:1;
        };
};

#endif
