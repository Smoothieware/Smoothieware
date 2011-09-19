/* mbed Microcontroller Library - PortOut
 * Copyright (c) 2006-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_PORTOUT_H
#define MBED_PORTOUT_H

#include "platform.h"
#include "PinNames.h"
#include "Base.h"

#include "PortNames.h"

namespace mbed {
/* Class: PortOut
 *   A multiple pin digital out
 *
 * Example:
 * > // Toggle all four LEDs
 * >
 * > #include "mbed.h"
 * >
 * > // LED1 = P1.18  LED2 = P1.20  LED3 = P1.21  LED4 = P1.23
 * > #define LED_MASK 0x00B40000
 * >
 * > PortOut ledport(Port1, LED_MASK);
 * >
 * > int main() {
 * >     while(1) {
 * >         ledport = LED_MASK;
 * >         wait(1);
 * >         ledport = 0;
 * >         wait(1);
 * >     }
 * > }
 */  
class PortOut {
public:

    /* Constructor: PortOut
     *  Create an PortOut, connected to the specified port
     *
     * Variables:
     *  port - Port to connect to (Port0-Port5)
     *  mask - A bitmask to identify which bits in the port should be included (0 - ignore)
   	 */ 
    PortOut(PortName port, int mask = 0xFFFFFFFF);

    /* Function: write
     *  Write the value to the output port
     *
     * Variables:
     *  value - An integer specifying a bit to write for every corresponding PortOut pin
     */    
    void write(int value) {
        _gpio->FIOPIN = (_gpio->FIOPIN & ~_mask) | (value & _mask);
    }

    /* Function: read
     *  Read the value currently output on the port
     *
     * Variables:
     *  returns - An integer with each bit corresponding to associated PortOut pin setting
     */
    int read() {
        return _gpio->FIOPIN & _mask;
    }

    /* Function: operator=
     *  A shorthand for <write>
     */    
    PortOut& operator= (int value) { 
    	write(value);
	    return *this;
    }
    
    PortOut& operator= (PortOut& rhs) { 
    	write(rhs.read());
	    return *this;
    }
    
    /* Function: operator int()
     *  A shorthand for <read>
     */
    operator int() { 
	    return read();
    }

private:
    LPC_GPIO_TypeDef    *_gpio;
    PortName            _port; 
    uint32_t            _mask;    
};

} // namespace mbed

#endif
