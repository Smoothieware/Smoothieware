/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/



#ifndef SLOWTICKER_H
#define SLOWTICKER_H

using namespace std;
#include <vector>
#include "mbed.h"
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/Hook.h"

class SlowTicker : public Module{
    public:
        SlowTicker();
        void set_frequency( int frequency );
        void tick();
        // For some reason this can't go in the .cpp, see :  http://mbed.org/forum/mbed/topic/2774/?page=1#comment-14221
        template<typename T> void attach( double frequency, T *optr, void ( T::*fptr )( void ) ){
            Hook* hook = new Hook(); 
            hook->frequency = frequency;
            hook->attach(optr, fptr);
            hook->counter = -2;
            if( frequency > this->max_frequency ){ 
                this->max_frequency = frequency; 
            } 
            this->set_frequency(this->max_frequency); 
            this->hooks.push_back(hook);
        }

        vector<Hook*> hooks; 
        double max_frequency;
};





#endif
