/* mbed Microcontroller Library - PortInOut
 * Copyright (c) 2006-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_PORTINOUT_H
#define MBED_PORTINOUT_H

#include "PortNames.h"
#include "PinNames.h"

namespace mbed {

/* Class: PortInOut
 *  A multiple pin digital in/out used to set/read multiple bi-directional pins
 */
class PortInOut {
public:

    /* Constructor: PortInOut
     *  Create an PortInOut, connected to the specified port
     *
     * Variables:
     *  port - Port to connect to (Port0-Port5)
     *  mask - A bitmask to identify which bits in the port should be included (0 - ignore)
   	 */ 
    PortInOut(PortName port, int mask = 0xFFFFFFFF);

    /* Function: write
     *  Write the value to the output port
     *
     * Variables:
     *  value - An integer specifying a bit to write for every corresponding port pin
     */    
    void write(int value) {
        _gpio->FIOPIN = (_gpio->FIOPIN & ~_mask) | (value & _mask);
    }

    /* Function: read
     *  Read the value currently output on the port
     *
     * Variables:
     *  returns - An integer with each bit corresponding to associated port pin setting
     */
    int read() {
        return _gpio->FIOPIN & _mask;
    }

    /* Function: output
     *  Set as an output
     */
    void output();

    /* Function: input
     *  Set as an input
     */
    void input();

    /* Function: mode
     *  Set the input pin mode
     *
     * Variables:
     *  mode - PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode mode);

    /* Function: operator=
     *  A shorthand for <write>
     */    
    PortInOut& operator= (int value) { 
    	write(value);
	    return *this;
    }
    
    PortInOut& operator= (PortInOut& rhs) { 
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
