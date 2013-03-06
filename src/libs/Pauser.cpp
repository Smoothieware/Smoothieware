#include "libs/Kernel.h"
#include "Pauser.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include <string>
using namespace std;

Pauser::Pauser(){}

void Pauser::on_module_loaded(){
    this->counter = 0;
}

void Pauser::take(){
    this->counter++;
    //this->kernel->streams->printf("take: %u \r\n", this->counter );
    if( this->counter == 1 ){
        this->kernel->call_event(ON_PAUSE, &this->counter);
    }
}

void Pauser::release(){
    this->counter--;
    //this->kernel->streams->printf("release: %u \r\n", this->counter );
    if( this->counter == 0 ){
        this->kernel->call_event(ON_PLAY, &this->counter);
    }
}

bool Pauser::paused()
{
    return (counter != 0);
}
