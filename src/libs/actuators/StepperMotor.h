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
#include "libs/actuators/Actuator.h"

class StepTicker;

class StepperMotor : public Actuator {
    public:
        //StepperMotor();
        StepperMotor(Pin* step, Pin* dir, Pin* en);

        void tick();
        void step();
        void move( bool direction, unsigned int steps );
        void signal_move_finished();
        void set_speed( double speed );
        void update_exit_tick();
        void pause();
        void unpause();
 
        StepTicker* step_ticker;
        Pin* step_pin;
        Pin* dir_pin;
        Pin* en_pin;

        bool is_move_finished; // Whether the move just finished
};


// Called a great many times per second, to step if we have to now
inline void StepperMotor::tick(){

    // increase the ( fixed point ) counter by one tick 11t
    this->fx_counter += (uint32_t)(1<<16);

    // if we are to step now 10t
    if( this->fx_counter >= this->fx_ticks_per_step ){ this->step(); }

}



#endif

