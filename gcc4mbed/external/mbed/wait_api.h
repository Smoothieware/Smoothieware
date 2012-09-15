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
 */ 
 
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

#ifdef TARGET_LPC11U24
/* Function: sleep
 *  Send the microcontroller to sleep
 * 
 * The processor is setup ready for sleep, and sent to sleep using __WFI(). In this mode, the
 * system clock to the core is stopped until a reset or an interrupt occurs. This eliminates 
 * dynamic power used by the processor, memory systems and buses. The processor, peripheral and 
 * memory state are maintained, and the peripherals continue to work and can generate interrupts.
 * 
 * The processor can be woken up by any internal peripheral interrupt or external pin interrupt.
 * 
 * Note: The mbed interface semihosting is disconnected as part of going to sleep, and can not be restored. 
 * Flash re-programming and the USB serial port will remain active, but the mbed program will no longer be
 * able to access the LocalFileSystem
 */
void sleep(void);

/* Function: deepsleep
 *  Send the microcontroller to deep sleep
 * 
 * This processor is setup ready for deep sleep, and sent to sleep using __WFI(). This mode
 * has the same sleep features as sleep plus it powers down peripherals and clocks. All state
 * is still maintained. 
 * 
 * The processor can only be woken up by an external interrupt on a pin or a watchdog timer.
 * 
 * Note: The mbed interface semihosting is disconnected as part of going to sleep, and can not be restored. 
 * Flash re-programming and the USB serial port will remain active, but the mbed program will no longer be
 * able to access the LocalFileSystem
 */
void deepsleep(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
