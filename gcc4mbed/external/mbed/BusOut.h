/* mbed Microcontroller Library - BusOut
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 * sford, rmeyer
 */
 
#ifndef MBED_BUSOUT_H
#define MBED_BUSOUT_H

#include "platform.h" 
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"
#include "DigitalOut.h"

namespace mbed {

/* Class: BusOut
 *  A digital output bus, used for setting the state of a collection of pins
 */
class BusOut : public Base {

public:

    /* Group: Configuration Methods */

    /* Constructor: BusOut
     *  Create an BusOut, connected to the specified pins
     *
     * Variables:
     *  p<n> - DigitalOut pin to connect to bus bit <n> (p5-p30, NC)
     *
     * Note:
     *  It is only required to specify as many pin variables as is required
     *  for the bus; the rest will default to NC (not connected)
   	 */ 
    BusOut(PinName p0, PinName p1 = NC, PinName p2 = NC, PinName p3 = NC,
           PinName p4 = NC, PinName p5 = NC, PinName p6 = NC, PinName p7 = NC,
           PinName p8 = NC, PinName p9 = NC, PinName p10 = NC, PinName p11 = NC,
           PinName p12 = NC, PinName p13 = NC, PinName p14 = NC, PinName p15 = NC, 
           const char *name = NULL);

    BusOut(PinName pins[16], const char *name = NULL);

    virtual ~BusOut();

    /* Group: Access Methods */
		
    /* Function: write
     *  Write the value to the output bus
     *
     * Variables:
     *  value - An integer specifying a bit to write for every corresponding DigitalOut pin
     */
    void write(int value);

		
    /* Function: read
     *  Read the value currently output on the bus
     *
     * Variables:
     *  returns - An integer with each bit corresponding to associated DigitalOut pin setting
     */
    int read();

#ifdef MBED_OPERATORS
    /* Group: Access Method Shorthand */
	   
   	/* Function: operator=
     *  A shorthand for <write>
     */
    BusOut& operator= (int v);
    BusOut& operator= (BusOut& rhs);

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

    DigitalOut* _pin[16];

#ifdef MBED_RPC
    static void construct(const char *arguments, char *res);
#endif
			
};

} // namespace mbed

#endif

