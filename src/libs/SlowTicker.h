/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef SLOWTICKER_H
#define SLOWTICKER_H

#include "Module.h"

using namespace std;
#include <vector>
#include <functional>

#include "libs/Pin.h"

#include "system_LPC17xx.h" // for SystemCoreClock
#include <math.h>

class SlowTicker : public Module{
  struct ticker {
    std::function<void(void)> f;
    int interval;
    int countdown;
    ticker(const std::function<void(void)>& _f,
           int i, int c) : f(_f), interval(i), countdown(c) {}
  };
    public:
        SlowTicker();

        void on_module_loaded(void);
        void on_idle(void*);

        void set_frequency( int frequency );
        void tick();
        // For some reason this can't go in the .cpp, see :  http://mbed.org/forum/mbed/topic/2774/?page=1#comment-14221
        // TODO replace this with std::function()
        void attach( uint32_t frequency, std::function<void(void)> f ){
            int interval = floorf((SystemCoreClock/4)/frequency);
            
            // to avoid race conditions we must stop the interupts before updating this non thread safe vector
            __disable_irq();
            if( frequency > this->max_frequency ){
                this->max_frequency = frequency;
                this->set_frequency(frequency);
            }
            this->hooks.emplace_back(f,interval, interval);
            __enable_irq();
            //            return hook;
        }

    private:
        bool flag_1s();

        vector<ticker> hooks;
        uint32_t max_frequency;
        uint32_t interval;

        Pin ispbtn;
protected:
    int flag_1s_count;
    volatile int flag_1s_flag;
};





#endif
