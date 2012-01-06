/* mbed Microcontroller Library - DigitalOut
 * Copyright (c) 2006-2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
#ifndef MBED_DIGITALOUT_H
#define MBED_DIGITALOUT_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: DigitalOut
 *  A digital output, used for setting the state of a pin
 *
 * Example:
 * > // Toggle a LED
 * > #include "mbed.h"
 * >
 * > DigitalOut led(LED1);
 * >
 * > int main() {
 * >     while(1) {
 * >         led = !led;
 * >         wait(0.2);
 * >     }
 * > }
 */
class DigitalOut : public Base {

public:

    /* Constructor: DigitalOut
     *  Create a DigitalOut connected to the specified pin
     *
     * Variables:
     *  pin - DigitalOut pin to connect to
     */
    DigitalOut(PinName pin, const char* name = NULL);

    /* Function: write
     *  Set the output, specified as 0 or 1 (int)
     *
     * Variables:
     *  value - An integer specifying the pin output value, 
     *      0 for logical 0 and 1 (or any other non-zero value) for logical 1 
     */
    void write(int value) {
        if(value) {
            _gpio->FIOSET = _mask;
        } else {
            _gpio->FIOCLR = _mask;
        }
    }

    /* Function: read
     *  Return the output setting, represented as 0 or 1 (int)
     *
     * Variables:
     *  returns - An integer representing the output setting of the pin, 
     *      0 for logical 0 and 1 for logical 1
     */
    int read() {
        return ((_gpio->FIOPIN & _mask) ? 1 : 0);
    }


#ifdef MBED_OPERATORS
    /* Function: operator=
     *  A shorthand for <write>
     */
    DigitalOut& operator= (int value) {
        write(value);
        return *this;
    }

    DigitalOut& operator= (DigitalOut& rhs) {
        write(rhs.read());
        return *this;
    }

    
    /* Function: operator int()
     *  A shorthand for <read>
     */
    operator int() {
        return read();
    }

#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    PinName             _pin;
    LPC_GPIO_TypeDef    *_gpio;
    uint32_t            _mask;


};

} // namespace mbed

#endif 
