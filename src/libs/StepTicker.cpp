/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/



using namespace std;
#include <vector>
#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepTicker.h"
#include "system_LPC17xx.h" // mbed.h lib


StepTicker* global_step_ticker;

StepTicker::StepTicker(){
    global_step_ticker = this;
    LPC_TIM0->MR0 = 1000000;        // Initial dummy value for Match Register
    LPC_TIM0->MCR = 11;              // Match on MR0, reset on MR0, match on MR1
    LPC_TIM0->TCR = 1;              // Enable interrupt
    NVIC_EnableIRQ(TIMER0_IRQn);    // Enable interrupt handler
}

void StepTicker::set_frequency( double frequency ){
    this->frequency = frequency;
    LPC_TIM0->MR0 = int(floor((SystemCoreClock/4)/frequency));  // SystemCoreClock/4 = Timer increments in a second
    if( LPC_TIM0->TC > LPC_TIM0->MR0 ){
        LPC_TIM0->TCR = 3;  // Reset
        LPC_TIM0->TCR = 1;  // Reset
    }
}

void StepTicker::set_reset_delay( double seconds ){
    LPC_TIM0->MR1 = int(floor(double(SystemCoreClock/4)*( seconds )));  // SystemCoreClock/4 = Timer increments in a second
}

void StepTicker::tick(){
    for (int i=0; i<this->hooks.size(); i++){ 
        this->hooks.at(i)->call();
    }
}

void StepTicker::reset_tick(){
    for (int i=0; i<this->reset_hooks.size(); i++){ 
        this->reset_hooks.at(i)->call();
    }
}

extern "C" void TIMER0_IRQHandler (void){
    if((LPC_TIM0->IR >> 0) & 1){  // If interrupt register set for MR0
        LPC_TIM0->IR |= 1 << 0;   // Reset it 
        global_step_ticker->tick(); 
    }
    if((LPC_TIM0->IR >> 1) & 1){  // If interrupt register set for MR1
        LPC_TIM0->IR |= 1 << 1;   // Reset it
        global_step_ticker->reset_tick();
    }
}


