/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DELTACALIBRATE_MODULE_H
#define DELTACALIBRATE_MODULE_H

#include "libs/Module.h"
#include "modules/communication/utils/Gcode.h"

#include "libs/StepperMotor.h"
#include "libs/Pin.h"

class DeltaCalibrate : public Module{
    public:
        DeltaCalibrate();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);


    private:
        void calibrate_delta(StreamOutput*);
        void calibrate_zprobe_offset(StreamOutput*);
        uint32_t wait_for_ztouch();
        uint32_t do_probe(bool);
        bool probe_towers(uint32_t[]);
        void wait_for_moves();
        void move_all(bool, bool, unsigned int);
        void move_all(bool, int[]);
        void trim2mm(float * mm);

        float steps_per_mm;
        float homing_position_z;
        bool home_direction[3];
        unsigned int  debounce_count;
        unsigned int  retract_steps;
        unsigned int lift_steps;
        float calibrate_radius;
        float calibrate_probe_offset;
        unsigned int max_passes;
        unsigned int calibrate_margin;
        

        int  trim[3];
        float  fast_rates;
        float  slow_rates;
        Pin           probe_pin;
        StepperMotor* steppers[3];
        bool is_delta;
        unsigned int delta_calibrate_flags;
        void send_gcode(std::string msg);
};

#endif
