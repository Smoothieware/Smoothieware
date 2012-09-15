/* mbed Microcontroller Library - Timeout
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_TIMEOUT_H
#define MBED_TIMEOUT_H

#include "Ticker.h"

namespace mbed {

/* Class: Timeout
 *  A Timeout is used to call a function at a point in the future
 *
 * You can use as many seperate Timeout objects as you require. 
 *
 * Example:
 * > // Blink until timeout.
 * >
 * > #include "mbed.h"
 * > 
 * > Timeout timeout;
 * > DigitalOut led(LED1);
 * > 
 * > int on = 1;
 * > 
 * > void attimeout() {
 * >     on = 0;
 * > }
 * > 
 * > int main() {
 * >     timeout.attach(&attimeout, 5);
 * >     while(on) {
 * >         led = !led;
 * >         wait(0.2);
 * >     }
 * > }
 */
class Timeout : public Ticker {

#if 0 // For documentation

    /* Function: attach
     *  Attach a function to be called by the Timeout, specifiying the delay in seconds
     *     
     * Variables:
     *  fptr - pointer to the function to be called
     *  t - the time before the call in seconds
     */
    void attach(void (*fptr)(void), float t) {
        attach_us(fptr, t * 1000000.0f);
    }
    
    /* Function: attach
     *  Attach a member function to be called by the Timeout, specifiying the delay in seconds
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     *  t - the time before the calls in seconds
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), float t) {
        attach_us(tptr, mptr, t * 1000000.0f);
    }
    
    /* Function: attach_us
     *  Attach a function to be called by the Timeout, specifiying the delay in micro-seconds
     *     
     * Variables:
     *  fptr - pointer to the function to be called
     *  t - the time before the call in micro-seconds
     */
    void attach_us(void (*fptr)(void), unsigned int t) {
        _function.attach(fptr);
        setup(t);
    }

    /* Function: attach_us
     *  Attach a member function to be called by the Timeout, specifiying the delay in micro-seconds
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     *  t - the time before the call in micro-seconds
     */    
    template<typename T>
    void attach_us(T* tptr, void (T::*mptr)(void), unsigned int t) {
        _function.attach(tptr, mptr);
        setup(t);
    }
    
    /* Function: detach
     *  Detach the function
     */        
    void detach();

#endif
    
protected:

    virtual void handler();

};

} // namespace mbed

#endif
