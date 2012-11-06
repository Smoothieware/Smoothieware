/* mbed Microcontroller Library - Timer
 * Copyright (c) 2007-2009 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_TIMER_H
#define MBED_TIMER_H

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: Timer
 *  A general purpose timer 
 *
 * Example:
 * > // Count the time to toggle a LED
 * >
 * > #include "mbed.h"
 * > 
 * > Timer timer;
 * > DigitalOut led(LED1);
 * > int begin, end;
 * > 
 * > int main() {
 * >     timer.start();
 * >     begin = timer.read_us();
 * >     led = !led;
 * >     end = timer.read_us();
 * >     printf("Toggle the led takes %d us", end - begin);
 * > }
 */
class Timer : public Base {

public:

    Timer(const char *name = NULL);
    
    /* Function: start
     *  Start the timer
     */
    void start(); 

    /* Function: stop
     *  Stop the timer
     */
    void stop(); 

    /* Function: reset
     *  Reset the timer to 0. 
     *
     * If it was already counting, it will continue
     */
    void reset();

    /* Function: read
     *  Get the time passed in seconds
     */
    float read();

    /* Function: read_ms
     *  Get the time passed in mili-seconds
     */
    int read_ms();

    /* Function: read_us
     *  Get the time passed in micro-seconds
     */
    int read_us();

#ifdef MBED_OPERATORS 
    operator float();
#endif

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
    static struct rpc_class *get_rpc_class();
#endif

protected:

    int slicetime();    
    int _running;          // whether the timer is running
    unsigned int _start;   // the start time of the latest slice
    int _time;             // any accumulated time from previous slices
    
};

} // namespace mbed

#endif

