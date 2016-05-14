/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "Pin.h"
#include <atomic>
#include <functional>

class StepTicker;
class Hook;

class StepperMotor {
    public:
        StepperMotor();
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        inline void step() { current_position_steps += (direction?-1:1); step_pin.set( 1 ); }
        inline void unstep() { step_pin.set(0); }
        inline void set_direction(bool f) { direction= f; dir_pin.set(f); }

        inline void enable(bool state) { en_pin.set(!state); };

        bool is_moving() const { return moving; }
        bool which_direction() const { return direction; }
//        void move_finished();
//        StepperMotor* move( bool direction, unsigned int steps, float initial_speed= -1.0F);
//        StepperMotor* set_speed( float speed );

        float get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        float get_last_milestone(void) const { return last_milestone_mm; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }

        int  steps_to_target(float);
        uint32_t get_stepped() const { return stepped; }
        void force_finish_move() { force_finish= true; }

        friend class StepTicker;
        friend class Stepper;
        friend class Planner;
        friend class Robot;

    private:
        void init();

        int index;

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        float steps_per_second;
        float steps_per_mm;
        float max_rate; // this is not really rate it is in mm/sec, misnamed used in Robot and Extruder
        float minimum_step_rate; // this is the minimum step_rate in steps/sec for this motor for this block
        static float default_minimum_actuator_rate;

        volatile int32_t current_position_steps;
        int32_t last_milestone_steps;
        float   last_milestone_mm;

        uint32_t stepped; // TBD may not be needed
        volatile struct {
            volatile bool is_move_finished:1; // Whether the move just finished
            volatile bool moving:1;
            volatile bool force_finish:1; // set to force a move to finish early
            bool direction:1;
        };
};

#endif

