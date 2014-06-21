/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Adc.h"
#include "libs/nuts_bolts.h"
#include "libs/Kernel.h"
#include "libs/Pin.h"
#include "libs/ADC/adc.h"
#include "libs/Pin.h"


using namespace std;
#include <vector>
// This is an interface to the mbed.org ADC library you can find in libs/ADC/adc.h
// TODO : Having the same name is confusing, should change that

Adc::Adc(){
    this->adc = new ADC(1000, 1);
}

// Enables ADC on a given pin
void Adc::enable_pin(Pin* pin){
    PinName pin_name = this->_pin_to_pinname(pin);
    this->adc->burst(1);
    this->adc->setup(pin_name,1);
    this->adc->interrupt_state(pin_name,1);
}

// Read the last value ( burst mode ) on a given pin
unsigned int Adc::read(Pin* pin){
    return this->adc->read(this->_pin_to_pinname(pin));
}

// Convert a smoothie Pin into a mBed Pin
PinName Adc::_pin_to_pinname(Pin* pin){
    if( pin->port == LPC_GPIO0 && pin->pin == 23 ){
        return p15;
    }else if( pin->port == LPC_GPIO0 && pin->pin == 24 ){
        return p16;
    }else if( pin->port == LPC_GPIO0 && pin->pin == 25 ){
        return p17;
    }else if( pin->port == LPC_GPIO0 && pin->pin == 26 ){
        return p18;
    }else if( pin->port == LPC_GPIO1 && pin->pin == 30 ){
        return p19;
    }else if( pin->port == LPC_GPIO1 && pin->pin == 31 ){
        return p20;
    }else{
        //TODO: Error
        return NC;
    }
}

