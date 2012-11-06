/* mbed Microcontroller Library - DigitalIn
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_DIGITALIN_H
#define MBED_DIGITALIN_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: DigitalIn
 *  A digital input, used for reading the state of a pin
 *
 * Example:
 * > // Flash an LED while a DigitalIn is true
 * >
 * > #include "mbed.h"
 * >
 * > DigitalIn enable(p5);
 * > DigitalOut led(LED1);
 * >
 * > int main() {
 * >     while(1) {
 * >         if(enable) {
 * >             led = !led;
 * >         }
 * >         wait(0.25);
 * >     }
 * > }
 */
class DigitalIn : public Base {

public:

    /* Constructor: DigitalIn
     *  Create a DigitalIn connected to the specified pin
     *
     * Variables:
     *  pin - DigitalIn pin to connect to
     *  name - (optional) A string to identify the object
     */
    DigitalIn(PinName pin, const char *name = NULL);

    /* Function: read
     *  Read the input, represented as 0 or 1 (int)
     *
     * Variables:
     *  returns - An integer representing the state of the input pin, 
     *      0 for logical 0 and 1 for logical 1
     */
    int read() {
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
        return ((_gpio->FIOPIN & _mask) ? 1 : 0);
#elif defined(TARGET_LPC11U24)
        return ((LPC_GPIO->PIN[_index] & _mask) ? 1 : 0);
#endif
    }


    /* Function: mode
     *  Set the input pin mode
     *
     * Variables:
     *  mode - PullUp, PullDown, PullNone, OpenDrain
     */
    void mode(PinMode pull);
    
#ifdef MBED_OPERATORS    
    /* Function: operator int()
     *  An operator shorthand for <read()>
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
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    LPC_GPIO_TypeDef    *_gpio;
#elif defined(TARGET_LPC11U24)
    int _index;
#endif
    uint32_t            _mask;

};

} // namespace mbed

#endif

