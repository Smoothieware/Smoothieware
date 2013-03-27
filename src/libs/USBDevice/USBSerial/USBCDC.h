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

#ifndef USBCDC_H
#define USBCDC_H

#include "USB.h"

/* These headers are included for child class. */
#include "USBEndpoints.h"
#include "USBDescriptor.h"
#include "USBDevice_Types.h"

#include "descriptor_cdc.h"

class USBCDC: public USB_Endpoint_Receiver {
public:
    USBCDC(USB *);

protected:
    bool send(uint8_t * buffer, uint32_t size);

    bool readEP(uint8_t * buffer, uint32_t * size);
    bool readEP_NB(uint8_t * buffer, uint32_t * size);

    virtual bool USBEvent_Request(CONTROL_TRANSFER&);
    virtual bool USBEvent_RequestComplete(CONTROL_TRANSFER&, uint8_t*, uint32_t);

    virtual void on_attach(void);
    virtual void on_detach(void);

    USB *usb;

    // USB Descriptors
    usbdesc_iad         CDC_iad;
    usbdesc_interface   CDC_if;
    usbcdc_header       CDC_header;
    usbcdc_callmgmt     CDC_callmgmt;
    usbcdc_acm          CDC_acm;
    usbcdc_union        CDC_union;

    usbdesc_interface   CDC_slaveif;

    usbdesc_endpoint    CDC_intep;
    usbdesc_endpoint    CDC_BulkIn;
    usbdesc_endpoint    CDC_BulkOut;

    usbdesc_string_l(15) CDC_string;

    usbcdc_line_coding  cdc_line_coding;
};

#endif
