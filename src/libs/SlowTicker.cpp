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
#include "modules/robot/Conveyor.h"
#include "Pauser.h"
#include "Gcode.h"

#include <mri.h>

// This module uses a Timer to periodically call hooks
// Modules register with a function ( callback ) and a frequency, and we then call that function at the given frequency.

SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    max_frequency = 0;
    global_slow_ticker = this;


    // ISP button FIXME: WHy is this here?
    ispbtn.from_string("2.10")->as_input()->pull_up();

    // TODO: What is this ??
    flag_1s_flag = 0;
    flag_1s_count = SystemCoreClock>>2;

    g4_ticks = 0;
    g4_pause = false;

    // Configure the actual timer after setup to avoid race conditions
    LPC_SC->PCONP |= (1 << 22);     // Power Ticker ON
    LPC_TIM2->MR0 = 10000;          // Initial dummy value for Match Register
    LPC_TIM2->MCR = 3;              // Match on MR0, reset on MR0
    LPC_TIM2->TCR = 1;              // Enable interrupt
    NVIC_EnableIRQ(TIMER2_IRQn);    // Enable interrupt handler
}

void SlowTicker::on_module_loaded(){
    register_for_event(ON_IDLE);
    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GCODE_EXECUTE);
}

// Set the base frequency we use for all sub-frequencies
void SlowTicker::set_frequency( int frequency ){
    this->interval = (SystemCoreClock >> 2) / frequency;   // SystemCoreClock/4 = Timer increments in a second
    LPC_TIM2->MR0 = this->interval;
    LPC_TIM2->TCR = 3;  // Reset
    LPC_TIM2->TCR = 1;  // Reset
    flag_1s_count= SystemCoreClock>>2;
}

// The actual interrupt being called by the timer, this is where work is done
void SlowTicker::tick(){

    // Call all hooks that need to be called ( bresenham )
    for (uint32_t i=0; i<this->hooks.size(); i++){
        Hook* hook = this->hooks.at(i);
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
        flag_1s_count += SystemCoreClock >> 2;
        // and set a flag for idle event to pick up
        flag_1s_flag++;
    }

    // if we're counting down a pause
    if (g4_ticks > 0)
    {
        // deduct tick time from timeout
        if (g4_ticks > interval)
            g4_ticks -= interval;
        else
            g4_ticks = 0;
    }

    // Enter MRI mode if the ISP button is pressed
    // TODO: This should have it's own module
    if (ispbtn.get() == 0)
        __debugbreak();

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

#include "gpio.h"
extern GPIO leds[];
void SlowTicker::on_idle(void*)
{
    static uint16_t ledcnt= 0;
    if(THEKERNEL->use_leds) {
        // flash led 3 to show we are alive
        leds[2]= (ledcnt++ & 0x1000) ? 1 : 0;
    }

    // if interrupt has set the 1 second flag
    if (flag_1s())
        // fire the on_second_tick event
        THEKERNEL->call_event(ON_SECOND_TICK);

    // if G4 has finished, release our pause
    if (g4_pause && (g4_ticks == 0))
    {
        g4_pause = false;
        THEKERNEL->pauser->release();
    }
}

// When a G4-type gcode is received, add it to the queue so we can execute it in time
void SlowTicker::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    // Add the gcode to the queue ourselves if we need it
    if( gcode->has_g && gcode->g == 4 ){
        THEKERNEL->conveyor->append_gcode(gcode);
        // ensure that no subsequent gcodes get executed along with our G4
        THEKERNEL->conveyor->queue_head_block();
    }
}

// When a G4-type gcode is executed, start the pause
void SlowTicker::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);

    if (gcode->has_g){
        if (gcode->g == 4){
            gcode->mark_as_taken();
            bool updated = false;
            if (gcode->has_letter('P')) {
                updated = true;
                g4_ticks += gcode->get_int('P') * ((SystemCoreClock >> 2) / 1000UL);
            }
            if (gcode->has_letter('S')) {
                updated = true;
                g4_ticks += gcode->get_int('S') * (SystemCoreClock >> 2);
            }
            if (updated){
                // G4 Smm Pnn should pause for mm seconds + nn milliseconds
                // at 120MHz core clock, the longest possible delay is (2^32 / (120MHz / 4)) = 143 seconds
                if (!g4_pause){
                    g4_pause = true;
                    THEKERNEL->pauser->take();
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

