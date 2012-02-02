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
