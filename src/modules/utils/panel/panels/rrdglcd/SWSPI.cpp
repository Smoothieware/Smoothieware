/* SWSPI, Software SPI library
 * Copyright (c) 2012-2014, David R. Van Wagner, http://techwithdave.blogspot.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "mbed.h"
#include "SWSPI.h"

SWSPI::SWSPI(PinName mosi_pin, PinName miso_pin, PinName sclk_pin)
    : mosi(mosi_pin),
      miso(NULL),
      sclk(sclk_pin)
{
    if (miso_pin!= NC)
        miso = new DigitalIn(miso_pin);
    format(8);
    frequency();
}

SWSPI::~SWSPI()
{
    delete miso;
}

void SWSPI::format(int bits, int mode)
{
    this->bits = bits;
    polarity = (mode >> 1) & 1;
    phase = mode & 1;
    sclk.write(polarity);
}

void SWSPI::frequency(int hz)
{
    // TODO: need wait_ns taking int
    this->delay_us = 500000/hz;
}

int SWSPI::write(int value)
{
    int read = 0;
    for (unsigned bit = 1 << (bits-1); bit; bit >>= 1)
    {
        mosi.write((value & bit) != 0);

        if (phase == 0)
        {
            if (miso && miso->read())
                read |= bit;
        }

        sclk.write(!polarity);

        if (delay_us) wait_us(delay_us);

        if (phase == 1)
        {
            if (miso && miso->read())
                read |= bit;
        }

        sclk.write(polarity);

        if (delay_us) wait_us(delay_us);
    }
    
    return read;
}
