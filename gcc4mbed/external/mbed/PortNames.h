/* mbed Microcontroller Library - PortName 
 * Copyright (c) 2010-2011 ARM Limited. All rights reserved.
 */

#ifndef MBED_PORTNAMES_H
#define MBED_PORTNAMES_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368)

enum PortName {
    Port0 = 0
    , Port1 = 1
    , Port2 = 2
    , Port3 = 3
    , Port4 = 4
};
typedef enum PortName PortName;

#elif defined(TARGET_LPC11U24)

enum PortName {
    Port0 = 0
    , Port1 = 1
};
typedef enum PortName PortName;


#endif

#ifdef __cplusplus
}
#endif
#endif

