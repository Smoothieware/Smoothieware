/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef USBENDPOINTS_H
#define USBENDPOINTS_H

#include "USBDevice_Types.h"

/* SETUP packet size */
#define SETUP_PACKET_SIZE (8)

/* Options flags for configuring endpoints */
#define DEFAULT_OPTIONS     (0)
#define SINGLE_BUFFERED     (1U << 0)
#define ISOCHRONOUS         (1U << 1)
#define RATE_FEEDBACK_MODE  (1U << 2) /* Interrupt endpoints only */

/* Endpoint transfer status, for endpoints > 0 */
typedef enum {
    EP_COMPLETED,   /* Transfer completed */
    EP_PENDING,     /* Transfer in progress */
    EP_INVALID,     /* Invalid parameter */
    EP_STALLED,     /* Endpoint stalled */
} EP_STATUS;

class USB_State_Receiver {
public:
    virtual bool USBEvent_busReset(void) = 0;
    virtual bool USBEvent_connectStateChanged(bool connected) = 0;
    virtual bool USBEvent_suspendStateChanged(bool suspended) = 0;
};

class USB_Frame_Receiver {
public:
    virtual bool USBEvent_Frame(uint16_t) = 0;
};

class USB_Class_Receiver {
public:
    virtual bool USBEvent_Request(CONTROL_TRANSFER&) = 0;
    virtual bool USBEvent_RequestComplete(CONTROL_TRANSFER&, uint8_t *buf, uint32_t length) = 0;
};

class USB_Endpoint_Receiver : public USB_Class_Receiver {
public:
    virtual bool USBEvent_EPIn(uint8_t, uint8_t) = 0;
    virtual bool USBEvent_EPOut(uint8_t, uint8_t) = 0;
};

/* Include configuration for specific target */
#if defined(TARGET_LPC1768) || defined(TARGET_LPC2368) || defined(__LPC17XX__)
    #include "USBEndpoints_LPC17_LPC23.h"
#elif defined(TARGET_LPC11U24)
    #include "USBEndpoints_LPC11U.h"
#else
    #error "Unknown target type"
#endif

#endif
