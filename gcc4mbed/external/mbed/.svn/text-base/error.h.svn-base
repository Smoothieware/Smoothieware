/* mbed Microcontroller Library - error
 * Copyright (c) 2006-2009 ARM Limited. All rights reserved.
 * sford
 */ 
 
#ifndef MBED_ERROR_H
#define MBED_ERROR_H

/* Reporting Compile-Time Errors:
 *  To generate a fatal compile-time error, you can use the pre-processor #error directive.
 *
 * > #error "That shouldn't have happened!"
 *
 * If the compiler evaluates this line, it will report the error and stop the compile.
 *
 * For example, you could use this to check some user-defined compile-time variables:
 *
 * > #define NUM_PORTS 7
 * > #if (NUM_PORTS > 4)
 * >     #error "NUM_PORTS must be less than 4"
 * > #endif
 *
 * Reporting Run-Time Errors:
 * To generate a fatal run-time error, you can use the mbed error() function.
 *
 * > error("That shouldn't have happened!");
 *
 * If the mbed running the program executes this function, it will print the 
 * message via the USB serial port, and then die with the blue lights of death!
 *
 * The message can use printf-style formatting, so you can report variables in the 
 * message too. For example, you could use this to check a run-time condition:
 * 
 * > if(x >= 5) {
 * >     error("expected x to be less than 5, but got %d", x);
 * > }
 */
 
#if 0 // for documentation only
/* Function: error
 * Report a fatal runtime error
 *
 * Outputs the specified error message to stderr so it will appear via the USB 
 * serial port, and then calls exit(1) to die with the blue lights of death.
 *
 * Variables:
 *  format - printf-style format string, followed by associated variables
 */
void error(const char* format, ...);
#endif  

#include <stdlib.h>

#ifdef NDEBUG
    #define error(...) (exit(1))
#else
    #include <stdio.h>
    #define error(...) (fprintf(stderr, __VA_ARGS__), exit(1))
#endif

#endif
