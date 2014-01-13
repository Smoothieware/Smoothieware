#include "libs/Kernel.h"
#include "Pauser.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;

// The Pauser module is the core of the pausing subsystem in smoothie. Basically we want several modules to be able to pause smoothie at the same time 
// ( think both the user with a button, and the temperature control because a temperature is not reached ). To do that, modules call the take() methode, 
// a pause event is called, and the pause does not end before all modules have called the release() method. 
// Please note : Modules should keep track of their pause status themselves
Pauser::Pauser(){}

void Pauser::on_module_loaded(){
    this->counter = 0;
}

// Pause smoothie if nobody else is currently doing so
void Pauser::take(){
    this->counter++;
    //THEKERNEL->streams->printf("take: %u \r\n", this->counter );
    if( this->counter == 1 ){
        THEKERNEL->call_event(ON_PAUSE, &this->counter);
    }
}

// Unpause smoothie unless something else is pausing it too
void Pauser::release(){
    this->counter--;
    //THEKERNEL->streams->printf("release: %u \r\n", this->counter );
    if( this->counter == 0 ){
        THEKERNEL->call_event(ON_PLAY, &this->counter);
    }
}

// Return wether smoothie is paused
bool Pauser::paused(){
    return (counter != 0);
}
