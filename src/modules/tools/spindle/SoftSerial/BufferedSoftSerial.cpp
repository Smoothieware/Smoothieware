/**
 * @file    BufferedSoftSerial.cpp
 * @brief   Software Buffer - Extends mbed Serial functionallity adding irq driven TX and RX
 * @author  sam grove
 * @version 1.0
 * @see
 *
 * Copyright (c) 2013
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "BufferedSoftSerial.h"
#include <stdarg.h>

BufferedSoftSerial::BufferedSoftSerial(PinName tx, PinName rx, const char* name)
    : SoftSerial(tx, rx, name)
{
    SoftSerial::attach(this, &BufferedSoftSerial::rxIrq, SoftSerial::RxIrq);

    return;
}

int BufferedSoftSerial::readable(void)
{
    return _rxbuf.size();  // note: look if things are in the buffer
}

int BufferedSoftSerial::writeable(void)
{
    return 1;   // buffer allows overwriting by design, always true
}

int BufferedSoftSerial::getc(void)
{
    char retval;
    _rxbuf.pop_front(retval);
    return (int)retval;
}

int BufferedSoftSerial::putc(int c)
{
    _txbuf.push_back((char)c);
    BufferedSoftSerial::prime();

    return c;
}

int BufferedSoftSerial::puts(const char *s)
{
    const char* ptr = s;

    while(*(ptr) != 0) {
        _txbuf.push_back(*(ptr++));
    }
    _txbuf.push_back('\n');  // done per puts definition
    BufferedSoftSerial::prime();

    return (ptr - s) + 1;
}

int BufferedSoftSerial::printf(const char* format, ...)
{
    char buf[256] = {0};
    int r = 0;

    va_list arg;
    va_start(arg, format);
    r = vsprintf(buf, format, arg);
    va_end(arg);
    r = BufferedSoftSerial::write(buf, r);

    return r;
}

ssize_t BufferedSoftSerial::write(const void *s, size_t length)
{
    const char* ptr = (const char*)s;
    const char* end = ptr + length;

    while (ptr != end) {
        _txbuf.push_back(*(ptr++));
    }
    BufferedSoftSerial::prime();

    return ptr - (const char*)s;
}


void BufferedSoftSerial::rxIrq(void)
{
    // read from the peripheral and make sure something is available
    if(SoftSerial::readable()) {
        _rxbuf.push_back(_getc()); // if so load them into a buffer
    }

    return;
}

void BufferedSoftSerial::txIrq(void)
{
    char retval;
    // see if there is room in the hardware fifo and if something is in the software fifo
    while(SoftSerial::writeable()) {
        if(_txbuf.size()) {
            _txbuf.pop_front(retval);
            _putc((int)retval);
        } else {
            // disable the TX interrupt when there is nothing left to send
            SoftSerial::attach(NULL, SoftSerial::TxIrq);
            break;
        }
    }

    return;
}

void BufferedSoftSerial::prime(void)
{
    // if already busy then the irq will pick this up
    if(SoftSerial::writeable()) {
        SoftSerial::attach(NULL, SoftSerial::TxIrq);    // make sure not to cause contention in the irq
        BufferedSoftSerial::txIrq();                // only write to hardware in one place
        SoftSerial::attach(this, &BufferedSoftSerial::txIrq, SoftSerial::TxIrq);
    }

    return;
}


