/* mbed Microcontroller Library - InterruptIn
 * Copyright (c) 2006-2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
#ifndef MBED_INTERRUPTIN_H
#define MBED_INTERRUPTIN_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"
#include "FunctionPointer.h"

namespace mbed {

/* Class: InterruptIn
 *  A digital interrupt input, used to call a function on a rising or falling edge
 *
 * Example:
 * > // Flash an LED while waiting for events
 * >
 * > #include "mbed.h"
 * >
 * > InterruptIn event(p16);
 * > DigitalOut led(LED1);
 * >
 * > void trigger() {
 * >     printf("triggered!\n");
 * > }
 * >
 * > int main() {
 * >     event.rise(&trigger);
 * >     while(1) {
 * >         led = !led;
 * >         wait(0.25);
 * >     }
 * > }
 */
class InterruptIn : public Base {

public:

    /* Constructor: InterruptIn
     *  Create an InterruptIn connected to the specified pin
     *
     * Variables:
     *  pin - InterruptIn pin to connect to
     *  name - (optional) A string to identify the object
     */
    InterruptIn(PinName pin, const char *name = NULL);
 
     int read();
#ifdef MBED_OPERATORS
    operator int();

#endif
     
    /* Function: rise
     *  Attach a function to call when a rising edge occurs on the input
     *
     * Variables:
     *  fptr - A pointer to a void function, or 0 to set as none
     */
    void rise(void (*fptr)(void));

    /* Function: rise
     *  Attach a member function to call when a rising edge occurs on the input
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     */
    template<typename T>
    void rise(T* tptr, void (T::*mptr)(void)) {
        _rise.attach(tptr, mptr);
        setup_interrupt(1, 1);
    }

    /* Function: fall
     *  Attach a function to call when a falling edge occurs on the input
     *
     * Variables:
     *  fptr - A pointer to a void function, or 0 to set as none
     */
    void fall(void (*fptr)(void));

    /* Function: fall
     *  Attach a member function to call when a falling edge occurs on the input
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     */
    template<typename T>
    void fall(T* tptr, void (T::*mptr)(void)) {
        _fall.attach(tptr, mptr);
        setup_interrupt(0, 1);
    }

    /* Function: mode
     *  Set the input pin mode
     *
     * Variables:
     *  mode - PullUp, PullDown, PullNone
     */
    void mode(PinMode pull);
    

 	static void _irq(); 
	static InterruptIn *_irq_objects[48];

protected:
	
    PinName _pin;
    FunctionPointer _rise;
    FunctionPointer _fall;

    void setup_interrupt(int rising, int enable);
    
};

} // namespace mbed

#endif
