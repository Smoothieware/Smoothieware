/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/



#ifndef STEPTICKER_H
#define STEPTICKER_H

using namespace std;
#include <vector>
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"


class StepTicker{
    public:
        StepTicker();
        void set_frequency( double frequency );
        void tick();
        void set_reset_delay( double seconds );
        void reset_tick();

        // For some reason this can't go in the .cpp, see :  http://mbed.org/forum/mbed/topic/2774/?page=1#comment-14221
        template<typename T> void attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            FPointer* hook = new FPointer(); 
            hook->attach(optr, fptr);
            this->hooks.push_back(hook);
        }

        template<typename T> void reset_attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            FPointer* reset_hook = new FPointer(); 
            reset_hook->attach(optr, fptr);
            this->reset_hooks.push_back(reset_hook);
        }


        vector<FPointer*> hooks; 
        vector<FPointer*> reset_hooks; 
        double frequency;

};











#endif
