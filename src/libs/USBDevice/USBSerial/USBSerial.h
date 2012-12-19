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

#ifndef USBSERIAL_H
#define USBSERIAL_H

#include "USBCDC.h"
// #include "Stream.h"
#include "CircBuffer.h"

#include "Module.h"
#include "StreamOutput.h"

class USBSerial_Receiver {
protected:
    virtual bool SerialEvent_RX(void) = 0;
};

class USBSerial: public USBCDC, public USBSerial_Receiver, public Module, public StreamOutput {
public:
    USBSerial(USB *);

    virtual int _putc(int c);
    virtual int _getc();

    uint8_t available();

    uint16_t writeBlock(uint8_t * buf, uint16_t size);

    CircBuffer<uint8_t> rxbuf;
    CircBuffer<uint8_t> txbuf;

    virtual void on_module_loaded(void);
    virtual void on_main_loop(void *);

protected:
//     virtual bool EpCallback(uint8_t, uint8_t);
    virtual bool USBEvent_EPIn(uint8_t, uint8_t);
    virtual bool USBEvent_EPOut(uint8_t, uint8_t);

    virtual bool SerialEvent_RX(void){return false;};

    virtual void on_attach(void);
    virtual void on_detach(void);

    volatile int nl_in_rx;
private:
    USB *usb;
//     mbed::FunctionPointer rx;
};

#endif
