/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "libs/Hook.h"

/* Actuator is the base class for all actuator-type things ( StepperMotors, ServoMotors etc ). */

class Actuator {
    public:
        Actuator(){};

        /* Functions all actuators must have are still to be defined, but here are a few bellow for a start */
        void tick();
        void step();
        void move_finished();
        void move( bool direction, unsigned int steps );
        void signal_move_finished();
        void set_speed( double speed );
        void update_exit_tick();
        void pause();
        void unpause();

        template<typename t> void attach( t *optr, uint32_t ( t::*fptr )( uint32_t ) ){
            Hook* hook = new Hook();
            hook->attach(optr, fptr);
            this->end_hook = hook;
        }

        template<typename t> void attach_signal_step(uint32_t step, t *optr, uint32_t ( t::*fptr )( uint32_t ) ){
            this->step_signal_hook->attach(optr, fptr);
            this->signal_step_number = step;
            this->signal_step = true;
        }

        Hook* end_hook;
        Hook* step_signal_hook;

        bool signal_step;
        uint32_t signal_step_number;




};



#endif
