/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Monitor for Remote Inspection. */
#ifndef _MRI_H_
#define _MRI_H_

#include <stdint.h>

/* Used to insert hardcoded breakpoint into user's code. */
#define __debugbreak()  { __asm volatile ("bkpt #0"); }

/* Error strings that can be returned to GDB. */
#define     MRI_ERROR_INVALID_ARGUMENT      "E01"   /* Encountered error when parsing command arguments. */
#define     MRI_ERROR_MEMORY_ACCESS_FAILURE "E03"   /* Couldn't access requested memory. */
#define     MRI_ERROR_BUFFER_OVERRUN        "E04"   /* Overflowed internal input/output buffer. */
#define     MRI_ERROR_NO_FREE_BREAKPOINT    "E05"   /* No free FPB breakpoint comparator slots. */


#ifdef __cplusplus
extern "C"
{
#endif


/* pDebuggerParameters string passed into __mriInit contains a space separated list of configuration parameters to be
   used to initialize the debug monitor.  The supported options include:
   
   One of these options to indicate which UART to be used for the debugger connection:
        MRI_UART_MBED_USB
        MRI_UART_MBED_P9_P10
        MRI_UART_MBED_P13_P14
        MRI_UART_MBED_P28_P27
        
    By default the debug monitor expects to take full control of the UART to configure baud rate, etc.  However 
    including the following option will tell the monitor to assume that the user's firmware will configure and use the
    serial port until the first exception occurs:
        MRI_UART_SHARE
*/
void __mriInit(const char* pDebuggerParameters);


/* Simple assembly language stubs that can be called from user's newlib _write and _read routines which will cause the 
   reads and writes to be redirected to the gdb host via mri. */
int __mriNewlib_SemihostWrite(int file, char *ptr, int len);
int __mriNewlib_SemihostRead(int file, char *ptr, int len);


#ifdef __cplusplus
}
#endif

#endif /* _MRI_H_ */


#ifndef MRI_VERSION_STRING

#define MRI_BRANCH "https://github.com/adamgreen/mri/tree/version_0.1"

#define MRI_VERSION_MAJOR       0
#define MRI_VERSION_MINOR       1
#define MRI_VERSION_BUILD       20120510
#define MRI_VERSION_SUBBUILD    1

#define MRI_STR(X) MRI_STR2(X)
#define MRI_STR2(X) #X

#define MRI_VERSION_STRING MRI_STR(MRI_VERSION_MAJOR) "." MRI_STR(MRI_VERSION_MINOR) "-" MRI_STR(MRI_VERSION_BUILD) "." MRI_STR(MRI_VERSION_SUBBUILD)

#endif
