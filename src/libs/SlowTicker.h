#ifndef SLOWTICKER_H
#define SLOWTICKER_H

using namespace std;
#include <vector>
#include "mbed.h"
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"

class SlowTicker{
    public:
        SlowTicker();
        void set_frequency( int frequency );
        void tick();
        // For some reason this can't go in the .cpp, see :  http://mbed.org/forum/mbed/topic/2774/?page=1#comment-14221
        template<typename T> void attach( T *optr, void ( T::*fptr )( void ) ){
            FunctionPointer* hook = new FunctionPointer(); 
            hook->attach(optr, fptr);
            this->hooks.push_back(hook);
        }

        vector<FunctionPointer*> hooks; 

};



















#endif
