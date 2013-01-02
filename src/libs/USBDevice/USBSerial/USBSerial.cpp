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

#include "USBSerial.h"

#include "libs/Kernel.h"
#include "libs/SerialMessage.h"

// extern void setled(int, bool);
#define setled(a, b) do {} while (0)

#define iprintf(...) do { } while (0)

USBSerial::USBSerial(USB *u): USBCDC(u), rxbuf(128), txbuf(128)
{
    usb = u;
    nl_in_rx = 0;
}

int USBSerial::_putc(int c)
{
//     send((uint8_t *)&c, 1);
    if (c == '\r')
        return 1;
    if (txbuf.free())
        txbuf.queue(c);

    usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
//     usb->endpointTriggerInterrupt(CDC_BulkIn.bEndpointAddress);
    return 1;
}

int USBSerial::_getc()
{
    uint8_t c = 0;
    setled(4, 1); while (rxbuf.isEmpty()); setled(4, 0);
    rxbuf.dequeue(&c);
    if (rxbuf.free() == MAX_PACKET_SIZE_EPBULK)
    {
        usb->endpointSetInterrupt(CDC_BulkOut.bEndpointAddress, true);
        iprintf("rxbuf has room for another packet, interrupt enabled\n");
//         usb->endpointTriggerInterrupt(CDC_BulkOut.bEndpointAddress);
    }
    if (nl_in_rx > 0)
        if (c == '\n')
            nl_in_rx--;

    return c;
}

int USBSerial::puts(const char *str)
{
    int i = 0;
    while (*str)
    {
        if ((*str != '\r') && txbuf.free())
            txbuf.queue(*str);
        if ((txbuf.available() % 64) == 0)
            usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
        i++;
        str++;
    }
    usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
    return i;
}

uint16_t USBSerial::writeBlock(uint8_t * buf, uint16_t size)
{
    if (size > txbuf.free())
    {
        size = txbuf.free();
    }
    if (size > 0)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            txbuf.queue(buf[i]);
        }
        usb->endpointSetInterrupt(CDC_BulkIn.bEndpointAddress, true);
    }
    return size;
}

bool USBSerial::USBEvent_EPIn(uint8_t bEP, uint8_t bEPStatus)
{
    /*
     * Called in ISR context
     */

//     static bool needToSendNull = false;

    bool r = true;

    if (bEP != CDC_BulkIn.bEndpointAddress)
        return false;

    iprintf("USBSerial:EpIn: 0x%02X\n", bEPStatus);

    uint8_t b[MAX_PACKET_SIZE_EPBULK];

    int l = txbuf.available();
    iprintf("%d bytes queued\n", l);
    if (l > 0)
    {
        if (l > MAX_PACKET_SIZE_EPBULK)
            l = MAX_PACKET_SIZE_EPBULK;
        iprintf("Sending %d bytes:\n\t", l);
        int i;
        for (i = 0; i < l; i++) {
            txbuf.dequeue(&b[i]);
            if (b[i] >= 32 && b[i] < 128)
                iprintf("%c", b[i]);
            else {
                iprintf("\\x%02X", b[i]);
            }
        }
        iprintf("\nSending...\n");
        send(b, l);
        iprintf("Sent\n");
//         if (l == 64 && txbuf.available() == 0)
//             needToSendNull = true;
        if (txbuf.available() == 0)
            r = false;
    }
    else
    {
//         if (needToSendNull)
//         {
//             send(NULL, 0);
//             needToSendNull = false;
//         }
//         else
//         {
            r = false;
//         }
//         usb->endpointSetInterrupt(bEP, false);
    }
    iprintf("USBSerial:EpIn Complete\n");
    return r;
}

bool USBSerial::USBEvent_EPOut(uint8_t bEP, uint8_t bEPStatus)
{
    /*
     * Called in ISR context
     */

    bool r = true;

    iprintf("USBSerial:EpOut\n");
    if (bEP != CDC_BulkOut.bEndpointAddress)
        return false;

    if (rxbuf.free() < MAX_PACKET_SIZE_EPBULK)
    {
//         usb->endpointSetInterrupt(bEP, false);
        return false;
    }

    uint8_t c[MAX_PACKET_SIZE_EPBULK];
    uint32_t size = 64;

    //we read the packet received and put it on the circular buffer
    readEP(c, &size);
    iprintf("Read %ld bytes:\n\t", size);
    for (uint8_t i = 0; i < size; i++) {
        rxbuf.queue(c[i]);
        if (c[i] >= 32 && c[i] < 128)
        {
            iprintf("%c", c[i]);
        }
        else
        {
            iprintf("\\x%02X", c[i]);
        }
        if (c[i] == '\n')
            nl_in_rx++;
    }
    iprintf("\nQueued, %d empty\n", rxbuf.free());

    if (rxbuf.free() < MAX_PACKET_SIZE_EPBULK)
    {
//         usb->endpointSetInterrupt(bEP, false);
        r = false;
    }

    usb->readStart(CDC_BulkOut.bEndpointAddress, MAX_PACKET_SIZE_EPBULK);
    iprintf("USBSerial:EpOut Complete\n");
    return r;
}

/*
bool USBSerial::EpCallback(uint8_t bEP, uint8_t bEPStatus) {
    if (bEP == CDC_BulkOut.bEndpointAddress) {
        uint8_t c[65];
        uint32_t size = 0;

        //we read the packet received and put it on the circular buffer
        readEP(c, &size);
        for (uint8_t i = 0; i < size; i++) {
            buf.queue(c[i]);
        }

        //call a potential handler
        rx.call();

        // We reactivate the endpoint to receive next characters
        usb->readStart(CDC_BulkOut.bEndpointAddress, MAX_PACKET_SIZE_EPBULK);
        return true;
    }
    return false;
}*/

uint8_t USBSerial::available()
{
    return rxbuf.available();
}

void USBSerial::on_module_loaded()
{
    this->register_for_event(ON_MAIN_LOOP);
//     this->kernel->streams->append_stream(this);
}

void USBSerial::on_main_loop(void *argument)
{
//     this->kernel->streams->printf("!");
    if (nl_in_rx)
    {
        string received;
        while (available())
        {
            char c = _getc();
            if( c == '\n' )
            {
                struct SerialMessage message;
                message.message = received;
                message.stream = this;
                iprintf("USBSerial Received: %s\n", message.message.c_str());
                this->kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                return;
            }
            else
            {
                received += c;
            }
        }
    }
}

void USBSerial::on_attach()
{
    this->kernel->streams->append_stream(this);
    writeBlock((uint8_t *) "Smoothie\nok\n", 12);
}

void USBSerial::on_detach()
{
    this->kernel->streams->remove_stream(this);
    txbuf.flush();
    rxbuf.flush();
    nl_in_rx = 0;
}
