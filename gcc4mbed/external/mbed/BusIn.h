/* mbed Microcontroller Library - DigitalIn
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */
 
#ifndef MBED_BUSIN_H
#define MBED_BUSIN_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"
#include "DigitalIn.h"

namespace mbed {

/* Class: BusIn
 *  A digital input bus, used for reading the state of a collection of pins
 */
class BusIn : public Base {

public:

    /* Group: Configuration Methods */

    /* Constructor: BusIn
     *  Create an BusIn, connected to the specified pins
     *
     * Variables:
     *  p<n> - DigitalIn pin to connect to bus bit <n> (p5-p30, NC)
     *
     * Note:
     *  It is only required to specify as many pin variables as is required
     *  for the bus; the rest will default to NC (not connected)
     */ 
    BusIn(PinName p0, PinName p1 = NC, PinName p2 = NC, PinName p3 = NC,
          PinName p4 = NC, PinName p5 = NC, PinName p6 = NC, PinName p7 = NC,
          PinName p8 = NC, PinName p9 = NC, PinName p10 = NC, PinName p11 = NC,
          PinName p12 = NC, PinName p13 = NC, PinName p14 = NC, PinName p15 = NC, 
          const char *name = NULL);

    BusIn(PinName pins[16], const char *name = NULL);
		
	virtual ~BusIn();
	
	/* Group: Access Methods */
		
	/* Function: read
	 *  Read the value of the input bus
	 *
	 * Variables:
	 *  returns - An integer with each bit corresponding to the value read from the associated DigitalIn pin
	 */
    int read();

#ifdef MBED_OPERATORS
    /* Group: Access Method Shorthand */
		
    /* Function: operator int()
     *  A shorthand for <read>
     */
    operator int();
#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:
	
    DigitalIn* _pin[16];

#ifdef MBED_RPC    
    static void construct(const char *arguments, char *res);
#endif

};

} // namespace mbed

#endif

