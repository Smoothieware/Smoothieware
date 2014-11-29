/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "libs/Hook.h"
#include "Pin.h"

class StepTicker;
class Hook;

class StepperMotor {
    public:
        StepperMotor();
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        void step();
        inline void unstep() { step_pin.set(0); };

        inline void enable(bool state) { en_pin.set(!state); };

        bool is_moving() { return moving; }
        void move_finished();
        void move( bool direction, unsigned int steps, float initial_speed= -1.0F);
        void signal_move_finished();
        void set_speed( float speed );
        void set_step_rate(float requested_rate, uint32_t block_steps_event_count);

        void update_exit_tick();
        void pause();
        void unpause();

        float get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        float get_last_milestone(void) const { return last_milestone_mm; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }
        float get_min_rate(void) const { return minimum_step_rate; }
        void set_min_rate(float mr) { minimum_step_rate= mr; }

        int  steps_to_target(float);
        uint32_t get_steps_to_move() const { return steps_to_move; }
        uint32_t get_stepped() const { return stepped; }

        template<typename T> void attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            Hook* hook = new Hook();
            hook->attach(optr, fptr);
            this->end_hook = hook;
        }

        friend class StepTicker;
        friend class Stepper;
        friend class Planner;
        friend class Robot;

    private:
        void init();

        int index;
        Hook* end_hook;

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

        uint32_t steps_to_move;
        uint32_t stepped;

        // set to 64 bit fixed point, 32:32 bits fractional
        static const uint32_t fx_shift= 32;
        static const uint64_t fx_increment= ((uint64_t)1<<fx_shift);
        volatile uint64_t fx_counter;
        volatile uint64_t fx_ticks_per_step;

        struct {
            bool direction:1;
            volatile bool is_move_finished:1; // Whether the move just finished
            bool paused:1;
            volatile bool moving:1;
        };

        // Called a great many times per second, to step if we have to now
        inline void tick() {
            // increase the ( 64 fixed point 32:32 ) counter by one tick 11t
            fx_counter += fx_increment;

            // if we are to step now 10t
            if (fx_counter >= fx_ticks_per_step)
                step();
        };
};

#endif

