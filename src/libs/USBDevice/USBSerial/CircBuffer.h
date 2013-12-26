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

#ifndef CIRCBUFFER_H
#define CIRCBUFFER_H

#include <stdlib.h>
#include "sLPC17xx.h"
#include "platform_memory.h"

template <class T>
class CircBuffer {
public:
    CircBuffer(int length) {
        write = 0;
        read = 0;
        size = length;
        buf = (uint8_t*) AHB0.alloc(size * sizeof(T));
    };

	bool isFull() {
		__disable_irq();
		bool b= ((write + 1) % size == read);
		__enable_irq();
		return b;
    };

    bool isEmpty() {
        return (read == write);
    };

    void queue(T k) {
		__disable_irq();
        if (isFull()) {
            read++;
            read %= size;
        }
        buf[write++] = k;
        write %= size;
		__enable_irq();
    }

    uint16_t available() {
		__disable_irq();
		uint16_t i= (write >= read) ? write - read : (size - read) + write;
		__enable_irq();
		return i;
    };
    uint16_t free() {
        return size - available() - 1;
    };

    void dump() {
        iprintf("[RingBuffer Sz:%2d Rd:%2d Wr:%2d Av:%2d Fr:%2d]\n", size, read, write, available(), free());
    }

    bool dequeue(T * c) {
        bool empty = isEmpty();
        if (!empty) {
            *c = buf[read++];
            read %= size;
        }
        return(!empty);
    };

    void peek(T * c, int offset) {
        int h = (read + offset) % size;
        *c = buf[h];
    };

    void flush() {
        read = write;
    }

private:
    volatile uint16_t write;
    volatile uint16_t read;
    uint16_t size;
    T * buf;
};

#endif
