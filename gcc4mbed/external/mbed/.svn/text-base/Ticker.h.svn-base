/* mbed Microcontroller Library - Ticker
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
#ifndef MBED_TICKER_H
#define MBED_TICKER_H

#include "TimerEvent.h"
#include "FunctionPointer.h"

namespace mbed {

/* Class: Ticker
 *  A Ticker is used to call a function at a recurring interval
 *
 * You can use as many seperate Ticker objects as you require. 
 *
 * Example:
 * > // Toggle the blinking led after 5 seconds
 * >
 * > #include "mbed.h"
 * > 
 * > Ticker timer;
 * > DigitalOut led1(LED1);
 * > DigitalOut led2(LED2);
 * > 
 * > int flip = 0;
 * > 
 * > void attime() {
 * >     flip = !flip;
 * > }
 * >
 * > int main() {
 * >     timer.attach(&attime, 5);
 * >     while(1) {
 * >         if(flip == 0) {
 * >             led1 = !led1;
 * >         } else {
 * >             led2 = !led2;
 * >         }
 * >         wait(0.2);
 * >     }
 * > }
 *
 */
class Ticker : public TimerEvent {

public:

    /* Function: attach
     *  Attach a function to be called by the Ticker, specifiying the interval in seconds
     *     
     * Variables:
     *  fptr - pointer to the function to be called
     *  t - the time between calls in seconds
     */
    void attach(void (*fptr)(void), float t) {
        attach_us(fptr, t * 1000000.0f);
    }
    
    /* Function: attach
     *  Attach a member function to be called by the Ticker, specifiying the interval in seconds
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     *  t - the time between calls in seconds
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), float t) {
        attach_us(tptr, mptr, t * 1000000.0f);
    }
    
    /* Function: attach_us
     *  Attach a function to be called by the Ticker, specifiying the interval in micro-seconds
     *     
     * Variables:
     *  fptr - pointer to the function to be called
     *  t - the time between calls in micro-seconds
     */
    void attach_us(void (*fptr)(void), unsigned int t) {
        _function.attach(fptr);
        setup(t);
    }

    /* Function: attach_us
     *  Attach a member function to be called by the Ticker, specifiying the interval in micro-seconds
     *     
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     *  t - the time between calls in micro-seconds
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

protected:

    void setup(unsigned int t);
    virtual void handler();

    unsigned int _delay;
    FunctionPointer _function;

};

} // namespace mbed

#endif
