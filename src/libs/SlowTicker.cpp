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
#include "SlowTicker.h"
#include "StepTicker.h"
#include "libs/Hook.h"
#include "modules/robot/Conveyor.h"
#include "Pauser.h"
#include "Gcode.h"

#include <mri.h>

#define SLOW_TIMER_PRESCALER    10000

// This module uses a Timer to periodically call hooks
// Modules register with a function ( callback ) and a frequency, and we then call that function at the given frequency.

SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    max_frequency = 0;
    global_slow_ticker = this;


    // ISP button FIXME: WHy is this here?
    //~ ispbtn.from_string("2.10")->as_input()->pull_up();

    // TODO: What is this ??
    flag_1s_flag = 0;
    // The APB1 bus clock is Core clock frequency / 2
    flag_1s_count = SystemCoreClock >> 1;
    // Tick 10 times per seconds at beginning
    this->interval = ((SystemCoreClock >> 1) / SLOW_TIMER_PRESCALER) / 10;

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;             // Enable TIM9 clock
    TIM4->PSC = SLOW_TIMER_PRESCALER - 1;           // Set prescaler to 10000
    TIM4->ARR = this->interval - 1;                 // Set auto-reload
    TIM4->EGR |= TIM_EGR_UG;                        // Force update
    TIM4->SR &= ~TIM_SR_UIF;                        // Clear the update flag
    TIM4->DIER |= TIM_DIER_UIE;                     // Enable interrupt on update event
    NVIC_EnableIRQ(TIM4_IRQn);                      // Enable TIM9 IRQ
    TIM4->CR1 |= TIM_CR1_CEN;                       // Enable TIM9 counter
    
}

void SlowTicker::on_module_loaded(){
    register_for_event(ON_IDLE);
}

// Set the base frequency we use for all sub-frequencies
void SlowTicker::set_frequency( int frequency ){
    this->interval = ((SystemCoreClock >> 1) / SLOW_TIMER_PRESCALER) / frequency;   // SystemCoreClock/4 = Timer increments in a second
    
    TIM4->CR1 &= ~TIM_CR1_CEN;                       // Disable TIM4 counter
    flag_1s_count = ((SystemCoreClock >> 1) / SLOW_TIMER_PRESCALER);
    TIM4->ARR = this->interval - 1;
    TIM4->CNT = 0;                       // Reset counter
    TIM4->CR1 |= TIM_CR1_CEN;                       // Enable it again
}

// The actual interrupt being called by the timer, this is where work is done
void SlowTicker::tick(){

    // Call all hooks that need to be called ( bresenham )
    for (Hook* hook : this->hooks){
        hook->countdown -= this->interval;
        if (hook->countdown < 0)
        {
            hook->countdown += hook->interval;
            hook->call();
        }
    }

    // deduct tick time from secound counter
    flag_1s_count -= this->interval;
    // if a whole second has elapsed,
    if (flag_1s_count < 0)
    {
        // add a second to our counter
        flag_1s_count += ((SystemCoreClock >> 1) / SLOW_TIMER_PRESCALER);
        // and set a flag for idle event to pick up
        flag_1s_flag++;
    }

    // Enter MRI mode if the ISP button is pressed
    // TODO: This should have it's own module
    //~ if (ispbtn.get() == 0)
        //~ __debugbreak();

}

bool SlowTicker::flag_1s(){
    // atomic flag check routine
    // first disable interrupts
    __disable_irq();
    // then check for a flag
    if (flag_1s_flag)
    {
        // if we have a flag, decrement the counter
        flag_1s_flag--;
        // re-enable interrupts
        __enable_irq();
        // and tell caller that we consumed a flag
        return true;
    }
    // if no flag, re-enable interrupts and return false
    __enable_irq();
    return false;
}

extern Pin leds[3];

void SlowTicker::on_idle(void*)
{
    // if interrupt has set the 1 second flag
    if (flag_1s()) {
        // fire the on_second_tick event
        THEKERNEL->call_event(ON_SECOND_TICK);
        if(THEKERNEL->use_leds) {
            // flash led 3 to show we are alive
            leds[2]= !leds[2];
        }
    }
}

extern "C" void TIM4_IRQHandler (void){
    if((TIM4->SR & TIM_SR_UIF) != 0)   { // If update flag is set
        TIM4->SR &= ~TIM_SR_UIF;         // Reset it
    }
    global_slow_ticker->tick();

}

