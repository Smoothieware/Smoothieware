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

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "USBCDC.h"

// static uint8_t cdc_line_coding[7]= {0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08};

#define iprintf(...) do { } while (0)

// #define DEFAULT_CONFIGURATION (1)

// #define CDC_SET_LINE_CODING        0x20
// #define CDC_GET_LINE_CODING        0x21
// #define CDC_SET_CONTROL_LINE_STATE 0x22

// #define MAX_CDC_REPORT_SIZE MAX_PACKET_SIZE_EPBULK

USBCDC::USBCDC(USB *u) {
    usb = u;

    CDC_iad = {
        DL_INTERFACE_ASSOCIATION,
        DT_INTERFACE_ASSOCIATION,
        0,                          // bFirstInterface - filled out later
        2,                          // bInterfaceCount - contiguous interfaces associated with this function
        UC_COMM,                    // bFunctionClass
        USB_CDC_SUBCLASS_ACM,       // bFunctionSubClass
        USB_CDC_PROTOCOL_ITU_V250,  // bFunctionProtocol
        0,                          // iFunction
    };
    CDC_if = {
        DL_INTERFACE,               // bLength
        DT_INTERFACE,               // bDescType
        0,                          // bInterfaceNumber: filled out during addInterface()
        0,                          // bAlternateSetting
        1,                          // bNumEndpoints
        UC_COMM,                    // bInterfaceClass
        USB_CDC_SUBCLASS_ACM,       // bInterfaceSubClass
        USB_CDC_PROTOCOL_ITU_V250,  // bInterfaceProtocol
        0,                          // iInterface
        0, 0, 0,                    // dummy padding
        this,                       // callback
    };
    CDC_intep = {
        DL_ENDPOINT,            // bLength
        DT_ENDPOINT,            // bDescType
        EP_DIR_IN,              // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_INTERRUPT,           // bmAttributes
        8,                      // wMaxPacketSize
        16,                     // bInterval
        0,                      // dummy padding
        this,                   // endpoint callback
    };
    CDC_header = {
        USB_CDC_LENGTH_HEADER,  // bLength
        DT_CDC_DESCRIPTOR,      // bDescType
        USB_CDC_SUBTYPE_HEADER, // bDescSubType
        0x0110,                 // bcdCDC
    };
    CDC_callmgmt = {
        USB_CDC_LENGTH_CALLMGMT,            // bLength
        DT_CDC_DESCRIPTOR,                  // bDescType
        USB_CDC_SUBTYPE_CALL_MANAGEMENT,    // bDescSubType
        USB_CDC_CALLMGMT_CAP_CALLMGMT | USB_CDC_CALLMGMT_CAP_DATAINTF,  // bmCapabilities
        0,                      // bDataInterface: filled in later
    };
    CDC_acm = {
        USB_CDC_LENGTH_ACM,     // bLength
        DT_CDC_DESCRIPTOR,      // bDescType
        USB_CDC_SUBTYPE_ACM,    // bDescSubType
        USB_CDC_ACM_CAP_LINE | USB_CDC_ACM_CAP_BRK, // bmCapabilities
    };
    CDC_union = {
        USB_CDC_LENGTH_UNION,   // bLength
        DT_CDC_DESCRIPTOR,      // bDescType
        USB_CDC_SUBTYPE_UNION,  // bDescSubType
        0,                      // bMasterInterface
        0,                      // bSlaveInterface0
    };
    CDC_slaveif = {
        DL_INTERFACE,           // bLength
        DT_INTERFACE,           // bDescType
        0,                      // bInterfaceNumber: filled out during addInterface()
        0,                      // bAlternateSetting
        2,                      // bNumEndpoints
        UC_CDC_DATA,            // bInterfaceClass
        0,                      // bInterfaceSubClass
        0,                      // bInterfaceProtocol
        0,                      // iInterface
        0, 0, 0,                // dummy padding
        this,                   // callback
    };
    CDC_BulkIn = {
        DL_ENDPOINT,            // bLength
        DT_ENDPOINT,            // bDescType
        EP_DIR_IN,              // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_BULK,                // bmAttributes
        MAX_PACKET_SIZE_EPBULK, // wMaxPacketSize
        1,                      // bInterval
        0,                      // dummy padding
        this,                   // endpoint callback
    };
    CDC_BulkOut = {
        DL_ENDPOINT,            // bLength
        DT_ENDPOINT,            // bDescType
        EP_DIR_OUT,             // bEndpointAddress: we provide direction, address is filled in by addEndpoint()
        EA_BULK,                // bmAttributes
        MAX_PACKET_SIZE_EPBULK, // wMaxPacketSize
        1,                      // bInterval
        0,                      // dummy padding
        this,                   // endpoint callback
    };

    usbdesc_string_l(16) s = usbstring("Smoothie Serial");
    memcpy(&CDC_string, &s, sizeof(CDC_string));

    usb->addDescriptor(&CDC_iad);
    uint8_t IfAddr =
        usb->addInterface(&CDC_if);
    usb->addDescriptor(&CDC_header);
    usb->addDescriptor(&CDC_callmgmt);
    usb->addDescriptor(&CDC_acm);
    usb->addDescriptor(&CDC_union);
    usb->addEndpoint(&CDC_intep);
    uint8_t slaveIfAddr =
        usb->addInterface(&CDC_slaveif);
    usb->addEndpoint(&CDC_BulkOut);
    usb->addEndpoint(&CDC_BulkIn);

    CDC_if.iInterface = usb->addString(&CDC_string);

    CDC_iad.bFirstInterface     = IfAddr;
    CDC_iad.iFunction           = CDC_if.iInterface;
    CDC_union.bMasterInterface  = IfAddr;
    CDC_union.bSlaveInterface0  = slaveIfAddr;
    CDC_callmgmt.bDataInterface = slaveIfAddr;

    cdc_line_coding.dwDTERate = 9600;
    cdc_line_coding.bCharFormat = 0;
    cdc_line_coding.bParityType = 0;
    cdc_line_coding.bDataBits   = 8;
}

/* Called in ISR context */
bool USBCDC::USBEvent_Request(CONTROL_TRANSFER &transfer) {

    /* Process class-specific requests */

    if (transfer.setup.bmRequestType.Type == CLASS_TYPE) {
        switch (transfer.setup.bRequest) {
            case CDC_GET_LINE_CODING:
//                 iprintf("[CDC]:GET_LINE_ENCODING\n");
                transfer.remaining = 7;
                transfer.ptr = (uint8_t *) &cdc_line_coding;
                transfer.direction = DEVICE_TO_HOST;
                return true;
            case CDC_SET_LINE_CODING:
//                 iprintf("[CDC]:SET_LINE_ENCODING\n");
                transfer.remaining = 7;
                transfer.ptr = (uint8_t *) &cdc_line_coding;
                transfer.notify = true;
                transfer.direction = HOST_TO_DEVICE;
                return true;
            case CDC_SET_CONTROL_LINE_STATE:
                iprintf("[CDC]:SET_CONTROL_LINE_STATE 0x%02X\n", transfer.setup.wValue);
                if (transfer.setup.wValue & CDC_CLS_DTR)
                    on_attach();
                else
                    on_detach();
                return true;
            default:
                break;
        }
    }

    return false;
}

bool USBCDC::USBEvent_RequestComplete(CONTROL_TRANSFER &transfer, uint8_t* buffer, uint32_t length)
{
    if (transfer.setup.bmRequestType.Type == CLASS_TYPE)
    {
        switch (transfer.setup.bRequest)
        {
            case CDC_SET_LINE_CODING:
            {
                iprintf("Got Line Coding:");
                iprintf(" BAUD: %lu ", cdc_line_coding.dwDTERate);
                iprintf(" STOP: ");
                    if (cdc_line_coding.bCharFormat == 0) iprintf("1");
                    else if (cdc_line_coding.bCharFormat == 1) iprintf("1.5");
                    else if (cdc_line_coding.bCharFormat == 2) iprintf("2");
                    else iprintf("?");
                iprintf(" PARITY: %d", cdc_line_coding.bParityType);
                iprintf(" DATABITS: %d", cdc_line_coding.bDataBits);

                iprintf("\n");
                return true;
            }
        }
    }
    return false;
}

bool USBCDC::send(uint8_t * buffer, uint32_t size) {
    return usb->writeNB(CDC_BulkIn.bEndpointAddress, buffer, size, CDC_BulkIn.wMaxPacketSize);
}

bool USBCDC::readEP(uint8_t * buffer, uint32_t * size) {
//     iprintf("USBCDC:readEP 0x%02X\n", CDC_BulkOut.bEndpointAddress);
    if (!usb->readEP(CDC_BulkOut.bEndpointAddress, buffer, size, CDC_BulkOut.wMaxPacketSize))
    {
//         iprintf("readEP failed\n");
        return false;
    }
//     iprintf("readEP ok\n");
    if (!usb->readStart(CDC_BulkOut.bEndpointAddress, CDC_BulkOut.wMaxPacketSize))
    {
//         iprintf("readStart failed\n");
        return false;
    }
//     iprintf("readStart ok\n");
    return true;
}

bool USBCDC::readEP_NB(uint8_t * buffer, uint32_t * size) {
    if (!usb->readEP_NB(CDC_BulkOut.bEndpointAddress, buffer, size, CDC_BulkOut.wMaxPacketSize))
        return false;
    if (!usb->readStart(CDC_BulkOut.bEndpointAddress, CDC_BulkOut.wMaxPacketSize))
        return false;
    return true;
}

void USBCDC::on_attach(void) {}

void USBCDC::on_detach(void) {}
