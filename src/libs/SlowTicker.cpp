using namespace std;
#include <vector>
#include "mbed.h"
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SlowTicker.h"


SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    global_slow_ticker = this;
    LPC_SC->PCONP |= (1 << 22);
    LPC_TIM2->MR0 = 1000000; 
    LPC_TIM2->MCR = 3;
    LPC_TIM2->TCR = 1; 
    NVIC_EnableIRQ(TIMER2_IRQn);
}

void SlowTicker::set_frequency( int frequency ){
    LPC_TIM2->MR0 = int(floor((SystemCoreClock/4)/frequency));
    LPC_TIM2->TCR = 3; 
    LPC_TIM2->TCR = 1; 
}

void SlowTicker::tick(){
    for (int i=0; i<this->hooks.size(); i++){ 
        this->hooks.at(i)->call();
    }
}

extern "C" void TIMER2_IRQHandler (void){
    if((LPC_TIM2->IR >> 0) & 1){
        LPC_TIM2->IR |= 1 << 0;
        global_slow_ticker->tick(); 
    }
}

