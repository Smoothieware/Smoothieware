/* mbed Microcontroller Library - InterruptIn
 * Copyright (c) 2006-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_INTERRUPTIN_H
#define MBED_INTERRUPTIN_H

#include "device.h"

#if DEVICE_INTERRUPTIN

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"
#include "FunctionPointer.h"

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
#define CHANNEL_NUM   48
#elif defined(TARGET_LPC11U24)
#define CHANNEL_NUM    8
#endif

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
#if defined(TARGET_LPC11U24)
    virtual ~InterruptIn();
#endif
 
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
    
    static InterruptIn *_irq_objects[CHANNEL_NUM];
    
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)
    static void _irq();
#elif defined(TARGET_LPC11U24)
    static void handle_interrupt_in(unsigned int channel);
    static void _irq0(); static void _irq1();
    static void _irq2(); static void _irq3();
    static void _irq4(); static void _irq5();
    static void _irq6(); static void _irq7();
#endif

protected:
    PinName _pin;
#if defined(TARGET_LPC11U24)
    Channel _channel;
#endif
    FunctionPointer _rise;
    FunctionPointer _fall;

    void setup_interrupt(int rising, int enable);
    
};

} // namespace mbed

#endif

#endif
