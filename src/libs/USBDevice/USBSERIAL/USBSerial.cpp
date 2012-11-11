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

#include "stdint.h"
#include "USBSerial.h"
#include "USBBusInterface.h"


int USBSerial::_putc(int c) {
    send((uint8_t *)&c, 1);
    return 1;
}

int USBSerial::_getc() {
    uint8_t c = 0;
    while (buf.isEmpty());
    buf.dequeue(&c);
    return c;
}


bool USBSerial::writeBlock(uint8_t * buf, uint16_t size) {
    if(size > MAX_PACKET_SIZE_EPBULK) {
        return false;
    }
    if(!send(buf, size)) {
        return false;
    }
    return true;
}



bool USBSerial::EP2_OUT_callback() {
    uint8_t c[65];
    uint16_t size = 0;

    //we read the packet received and put it on the circular buffer
    readEP(c, &size);
    for (int i = 0; i < size; i++) {
        buf.queue(c[i]);
    }

    //call a potential handler
    rx.call();

    // We reactivate the endpoint to receive next characters
    readStart(EPBULK_OUT, MAX_PACKET_SIZE_EPBULK);
    return true;
}

uint8_t USBSerial::available() {
    return buf.available();
}

