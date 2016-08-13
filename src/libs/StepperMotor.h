/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"
#include "Pin.h"

class StepperMotor  : public Module {
    public:
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        // called from step ticker ISR
        inline bool step() { step_pin.set(1); current_position_steps += (direction?-1:1); return moving; }
        // called from unstep ISR
        inline void unstep() { step_pin.set(0); }
        // called from step ticker ISR
        inline void set_direction(bool f) { dir_pin.set(f); direction= f; }
        inline bool same_direction(bool f) { return direction==f; }
        inline void enable(bool state) { en_pin.set(!state); };
        inline bool is_enabled() const { return !en_pin.get(); };
        inline bool is_moving() const { return moving; };
        void start_moving() { moving= true; }
        void stop_moving() { moving= false; }

        void manual_step(bool dir);

        bool which_direction() const { return direction; }

        //float get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        void set_last_milestones(float, int32_t);
        void update_last_milestones(float mm, int32_t steps);
        float get_last_milestone(void) const { return last_milestone_mm; }
        int32_t get_last_milestone_steps(void) const { return last_milestone_steps; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        int32_t get_current_step(void) const { return current_position_steps; }
        int32_t get_steps_per_tick(void) const { return steps_per_tick; }
        void set_steps_per_tick( int32_t st ) { steps_per_tick= st; }
        void update_steps_per_tick( int32_t ac ) { steps_per_tick+= ac; }
        int32_t get_counter(void) const { return counter; }
        void set_counter( int32_t c ) { counter= c; }
        void update_counter(void) { counter+=steps_per_tick; }
        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }
        void set_acceleration(float a) { acceleration= a; }
        float get_acceleration() const { return acceleration; }
        bool is_selected() const { return selected; }
        void set_selected(bool b) { selected= b; }

        int32_t steps_to_target(float);


    private:
        void on_halt(void *argument);
        void on_enable(void *argument);

        int index;

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        //float steps_per_second;
        float steps_per_mm;
        float max_rate; // this is not really rate it is in mm/sec, misnamed used in Robot and Extruder
        float acceleration;

        volatile int32_t current_position_steps;
        volatile int32_t steps_per_tick; // 2.30 fixed point, aka current speed
        volatile int32_t counter; // 2.30 fixed point, 0.0 .. 1.0 partial step
        int32_t last_milestone_steps;
        float   last_milestone_mm;

        volatile struct {
            volatile bool direction:1;
            volatile bool moving:1;
            bool selected:1;
        };
};

