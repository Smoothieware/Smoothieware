/* mbed Microcontroller Library - AnalogOut
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_ANALOGOUT_H
#define MBED_ANALOGOUT_H

#include "device.h"

#if DEVICE_ANALOGOUT

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: AnalogOut
 *  An analog output, used for setting the voltage on a pin
 *
 * Example:
 * > // Make a sawtooth output
 * > 
 * > #include "mbed.h"
 * > 
 * > AnalogOut tri(p18);
 * > int main() {
 * >     while(1) {
 * >         tri = tri + 0.01;
 * >         wait_us(1);
 * >         if(tri == 1) {
 * >             tri = 0;
 * >         }
 * >     }
 * > }
 */
class AnalogOut : public Base {

public:

	/* Constructor: AnalogOut
	 *  Create an AnalogOut connected to the specified pin
	 * 
	 * Variables:
	 *  pin - AnalogOut pin to connect to (18)
	 */
	AnalogOut(PinName pin, const char *name = NULL);
	
    /* Function: write
     *  Set the output voltage, specified as a percentage (float)
     *
     * Variables:
     *  percent - A floating-point value representing the output voltage, 
     *    specified as a percentage. The value should lie between
     *    0.0f (representing 0v / 0%) and 1.0f (representing 3.3v / 100%).
     *    Values outside this range will be saturated to 0.0f or 1.0f.	 
     */
    void write(float value);
    
    /* Function: write_u16
     *  Set the output voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
     *
     * Variables:
     *  value - 16-bit unsigned short representing the output voltage,
     *            normalised to a 16-bit value (0x0000 = 0v, 0xFFFF = 3.3v)
     */
    void write_u16(unsigned short value);

    /* Function: read
     *  Return the current output voltage setting, measured as a percentage (float)
     *
     * Variables:
     *  returns - A floating-point value representing the current voltage being output on the pin, 
     *    measured as a percentage. The returned value will lie between
     *    0.0f (representing 0v / 0%) and 1.0f (representing 3.3v / 100%).
     *
     * Note:
     *  This value may not match exactly the value set by a previous <write>.
     */	
    float read();


#ifdef MBED_OPERATORS
    /* Function: operator=
     *  An operator shorthand for <write()>
     */
	AnalogOut& operator= (float percent);
	AnalogOut& operator= (AnalogOut& rhs);

    /* Function: operator float()
     *  An operator shorthand for <read()>
     */	
	operator float();
#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

	DACName _dac;

};

} // namespace mbed

#endif

#endif
