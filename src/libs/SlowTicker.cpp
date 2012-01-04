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
    LPC_SC->PCONP |= (1 << 22);     // Power Ticker ON
    LPC_TIM2->MR0 = 1000000;        // Initial dummy value for Match Register
    LPC_TIM2->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM2->TCR = 1;              // Enable interrupt
    NVIC_EnableIRQ(TIMER2_IRQn);    // Enable interrupt handler
}

void SlowTicker::set_frequency( int frequency ){
    LPC_TIM2->MR0 = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM2->TCR = 3;  // Reset
    LPC_TIM2->TCR = 1;  // Reset
}

void SlowTicker::tick(){
    for (int i=0; i<this->hooks.size(); i++){ 
        this->hooks.at(i)->call();
    }
}

extern "C" void TIMER2_IRQHandler (void){
    if((LPC_TIM2->IR >> 0) & 1){  // If interrupt register set for MR0
        LPC_TIM2->IR |= 1 << 0;   // Reset it 
        global_slow_ticker->tick(); 
    }
}

