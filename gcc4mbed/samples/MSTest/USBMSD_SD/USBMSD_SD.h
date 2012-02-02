/* mbed USBMSD_SD Library, for providing file access to SD cards
 * Copyright (c) 2008-2010, sford
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

#ifndef USBMSD_SD_H
#define USBMSD_SD_H

#include "mbed.h"
#include "USBMSD.h"

/** Use the SDcard as mass storage device using the USBMSD class
 *
 * @code
 * #include "mbed.h"
 * #include "USBMSD_SD.h"
 *
 * USBMSD_SD sd(p5, p6, p7, p8);
 *
 * int main() {
 *   while(1);
 * }
 *
 * @endcode
 */
class USBMSD_SD : public USBMSD {
public:

    /** Create the File System for accessing an SD Card using SPI
     *
     * @param mosi SPI mosi pin connected to SD Card
     * @param miso SPI miso pin conencted to SD Card
     * @param sclk SPI sclk pin connected to SD Card
     * @param cs   DigitalOut pin used as SD Card chip select
     * @param name The name used to access the virtual filesystem
     */
    USBMSD_SD(PinName mosi, PinName miso, PinName sclk, PinName cs);
    virtual int disk_initialize();
    virtual int disk_write(const char *buffer, int block_number);
    virtual int disk_read(char *buffer, int block_number);    
    virtual int disk_status();
    virtual int disk_sync();
    virtual int disk_sectors();
    virtual int disk_size();

protected:

    int _cmd(int cmd, int arg);
    int _cmdx(int cmd, int arg);
    int _cmd8();
    int _cmd58();
    int initialise_card();
    int initialise_card_v1();
    int initialise_card_v2();
    
    int _read(char *buffer, int length);
    int _write(const char *buffer, int length);
    int _sd_sectors();
    int _sectors;
    
    int capacity;
    int _status;
    
    SPI _spi;
    DigitalOut _cs;
};

#endif
