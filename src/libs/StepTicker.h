/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef STEPTICKER_H
#define STEPTICKER_H

#include <stdint.h>

class StepperMotor;

class StepTicker{
    public:
        friend class StepperMotor;
        static StepTicker* global_step_ticker;

        StepTicker(int nmotors);
        ~StepTicker();
        void set_frequency( float frequency );
        void tick();
        void signal_a_move_finished();
        void set_reset_delay( float seconds );
        void reset_tick();
        void add_motor_to_active_list(StepperMotor* motor);
        void remove_motor_from_active_list(StepperMotor* motor);
        void TIMER0_IRQHandler (void);
        void PendSV_IRQHandler (void);

    private:
        float frequency;
        uint32_t delay;
        uint32_t period;
        uint32_t last_duration;

        StepperMotor** active_motors;
        uint32_t active_motor_bm; // limit to 32 motors
        struct {
            uint8_t num_motors:6; // increase for more than 32 motors
            bool a_move_finished:1;
            bool reset_step_pins:1;
        };

};



#endif
