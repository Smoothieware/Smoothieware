/* Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Monitor for Remote Inspection header file. */
#ifndef _MRI_H_
#define _MRI_H_

#include <stdint.h>

/* Used to insert hardcoded breakpoint into user's code. */
#define __debugbreak()  { __asm volatile ("bkpt #0"); }

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


#ifdef __cplusplus
}
#endif

#endif /* _MRI_H_ */
