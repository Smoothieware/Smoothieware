/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "libs/Kernel.h"
#include "libs/Hook.h"

class StepTicker;

class StepperMotor {
    public:
        StepperMotor();
        StepperMotor(Pin* step, Pin* dir, Pin* en);
        bool tick();
        void move( bool direction, unsigned int steps );
        void set_speed( double speed );
        void update_exit_tick();

        template<typename T> void attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            Hook* hook = new Hook(); 
            hook->attach(optr, fptr);
            this->end_hook = hook;
        }

        Hook* end_hook;

        StepTicker* step_ticker;
        Pin* step_pin;
        Pin* dir_pin;
        Pin* en_pin;

        double steps_per_second;

        bool moving;
        bool paused;

        bool direction_bit;
        bool step_bit;

        uint32_t steps_to_move;
        uint32_t stepped;
        uint64_t fx_counter;
        uint64_t fx_ticks_per_step;
        
        bool exit_tick;
        bool dont_remove_from_active_list_yet;
};



#endif

