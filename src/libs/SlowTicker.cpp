using namespace std;
#include <vector>
#include "mbed.h"
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SlowTicker.h"
#include "libs/Hook.h"


SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    this->max_frequency = 1;
    global_slow_ticker = this;
    LPC_SC->PCONP |= (1 << 22);     // Power Ticker ON
    LPC_TIM2->MR0 = 10000;        // Initial dummy value for Match Register
    LPC_TIM2->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM2->TCR = 1;              // Enable interrupt
    NVIC_EnableIRQ(TIMER2_IRQn);    // Enable interrupt handler
}

void SlowTicker::set_frequency( int frequency ){
    //this->kernel->serial->printf("setting:%d size:%d \r\n", frequency, this->hooks.size() ); 
    LPC_TIM2->MR0 = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM2->TCR = 3;  // Reset
    LPC_TIM2->TCR = 1;  // Reset
}

void SlowTicker::tick(){
    if( this->max_frequency < 0.1 ){
        this->kernel->serial->printf("empty\r\n");
    }
    for (int i=0; i<this->hooks.size(); i++){ 
        Hook* hook = this->hooks.at(i);
        hook->counter += ( hook->frequency / this->max_frequency );
        //this->kernel->serial->printf("%p cand size:%d, hook->freq:%f this->max:%f hook->counter:%f added:%f \r\n", hook, this->hooks.size(), hook->frequency, this->max_frequency, hook->counter, ( hook->frequency / this->max_frequency ) ); 
        if( hook->counter > 0 ){
            hook->counter-=1;
            //this->kernel->serial->printf("%p exec size:%d, hook->freq:%f this->max:%f hook->counter:%f added:%f \r\n", hook, this->hooks.size(), hook->frequency, this->max_frequency, hook->counter, ( hook->frequency / this->max_frequency ) ); 
            hook->call();
        } 
    }
}

extern "C" void TIMER2_IRQHandler (void){
    if((LPC_TIM2->IR >> 0) & 1){  // If interrupt register set for MR0
        LPC_TIM2->IR |= 1 << 0;   // Reset it 
        global_slow_ticker->tick(); 
    }
}

