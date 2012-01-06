#ifndef STEPTICKER_H
#define STEPTICKER_H

using namespace std;
#include <vector>
#include "mbed.h"
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
        template<typename T> void attach( T *optr, void ( T::*fptr )( void ) ){
            FunctionPointer* hook = new FunctionPointer(); 
            hook->attach(optr, fptr);
            this->hooks.push_back(hook);
        }

        template<typename T> void reset_attach( T *optr, void ( T::*fptr )( void ) ){
            FunctionPointer* reset_hook = new FunctionPointer(); 
            reset_hook->attach(optr, fptr);
            this->reset_hooks.push_back(reset_hook);
        }


        vector<FunctionPointer*> hooks; 
        vector<FunctionPointer*> reset_hooks; 
        double frequency;

};











#endif
