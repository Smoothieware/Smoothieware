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
#include "libs/Hook.h"

#include <mri.h>

SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    max_frequency = 0;
    global_slow_ticker = this;
    LPC_SC->PCONP |= (1 << 22);     // Power Ticker ON
    LPC_TIM2->MR0 = 10000;        // Initial dummy value for Match Register
    LPC_TIM2->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM2->TCR = 1;              // Enable interrupt
    NVIC_EnableIRQ(TIMER2_IRQn);    // Enable interrupt handler

    ispbtn.from_string("2.10")->as_input()->pull_up();

    flag_1s_flag = 0;
    flag_1s_count = SystemCoreClock;

    g4_ticks = 0;
    g4_pause = false;
}

void SlowTicker::on_module_loaded()
{
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_EXECUTE);
}

void SlowTicker::set_frequency( int frequency ){
    this->interval = (SystemCoreClock >> 2) / frequency;   // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM2->MR0 = this->interval;
    LPC_TIM2->TCR = 3;  // Reset
    LPC_TIM2->TCR = 1;  // Reset
}

void SlowTicker::tick()
{
    _isr_context = true;

    LPC_GPIO1->FIODIR |= 1<<20;
    LPC_GPIO1->FIOSET = 1<<20;

    for (uint32_t i=0; i<this->hooks.size(); i++){
        Hook* hook = this->hooks.at(i);
        hook->countdown -= this->interval;
        if (hook->countdown < 0)
        {
            hook->countdown += hook->interval;
            hook->call();
        }
    }

    flag_1s_count -= this->interval;
    if (flag_1s_count < 0)
    {
        flag_1s_count += SystemCoreClock >> 2;
        flag_1s_flag++;
    }

    if (g4_ticks > 0)
    {
        if (g4_ticks > interval)
            g4_ticks -= interval;
        else
            g4_ticks = 0;
    }

    LPC_GPIO1->FIOCLR = 1<<20;

    if (ispbtn.get() == 0)
        __debugbreak();

    _isr_context = false;
}

bool SlowTicker::flag_1s(){
    __disable_irq();
    if (flag_1s_flag)
    {
        flag_1s_flag--;
        __enable_irq();
        return true;
    }
    __enable_irq();
    return false;
}

void SlowTicker::on_idle(void*)
{
    if (flag_1s())
        kernel->call_event(ON_SECOND_TICK);

    // if G4 has finished, release our pause
    if (g4_pause && (g4_ticks == 0))
    {
        g4_pause = false;
        kernel->pauser->release();
    }
}

void SlowTicker::on_gcode_execute(void* argument)
{
    Gcode* gcode = static_cast<Gcode*>(argument);

    if (gcode->has_g)
    {
        if (gcode->g == 4)
        {
            bool updated = false;
            if (gcode->has_letter('P')) {
                updated = true;
                g4_ticks += gcode->get_int('P') * ((SystemCoreClock >> 2) / 1000UL);
            }
            if (gcode->has_letter('S')) {
                updated = true;
                g4_ticks += gcode->get_int('S') * (SystemCoreClock >> 2);
            }
            if (updated)
            {
                // G4 Smm Pnn should pause for mm seconds + nn milliseconds
                // at 120MHz core clock, the longest possible delay is (2^32 / (120MHz / 4)) = 143 seconds
                if (!g4_pause)
                {
                    g4_pause = true;
                    kernel->pauser->take();
                }
            }
        }
    }
}

extern "C" void TIMER2_IRQHandler (void){
    if((LPC_TIM2->IR >> 0) & 1){  // If interrupt register set for MR0
        LPC_TIM2->IR |= 1 << 0;   // Reset it
    }
    global_slow_ticker->tick();
}

