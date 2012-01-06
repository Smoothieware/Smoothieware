/* Title: wait
 * Generic wait functions.
 *
 * These provide simple NOP type wait capabilities.
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > DigitalOut heartbeat(LED1);
 * >
 * > int main() {
 * >     while (1) {
 * >         heartbeat = 1;
 * >         wait(0.5);
 * >         heartbeat = 0;
 * >         wait(0.5);
 * >     }
 * > }
 */

/* mbed Microcontroller Library - wait_api
 * Copyright (c) 2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
// GENERIC

#ifndef MBED_WAIT_API_H
#define MBED_WAIT_API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Function: wait
 *  Waits for a number of seconds, with microsecond resolution (within
 *  the accuracy of single precision floating point).
 *
 * Variables:
 *  s - number of seconds to wait
 */
void wait(float s);

/* Function: wait_ms
 *  Waits a number of milliseconds.
 *
 * Variables:
 *  ms - the whole number of milliseconds to wait
 */
void wait_ms(int ms);

/* Function: wait_us
 *  Waits a number of microseconds.
 *
 * Variables:
 *  us - the whole number of microseconds to wait
 */
void wait_us(int us);

#ifdef __cplusplus
}
#endif

#endif
