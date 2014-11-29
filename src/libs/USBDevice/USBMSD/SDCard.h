/* mbed SDFileSystem Library, for providing file access to SD cards
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
 *
 *
 * This version significantly altered by Michael Moon and is (c) 2012
 */

#ifndef SDCARD_H
#define SDCARD_H

#include "spi.h"
#include "gpio.h"

#include "disk.h"
#include "mbed.h"

// #include "DMA.h"

/** Access the filesystem on an SD Card using SPI
 *
 * @code
 * #include "mbed.h"
 * #include "SDFileSystem.h"
 *
 * SDFileSystem sd(p5, p6, p7, p12, "sd"); // mosi, miso, sclk, cs
 *
 * int main() {
 *     FILE *fp = fopen("/sd/myfile.txt", "w");
 *     fprintf(fp, "Hello World!\n");
 *     fclose(fp);
 * }
 */
class SDCard : public MSD_Disk {
public:

    /** Create the File System for accessing an SD Card using SPI
     *
     * @param mosi SPI mosi pin connected to SD Card
     * @param miso SPI miso pin conencted to SD Card
     * @param sclk SPI sclk pin connected to SD Card
     * @param cs   DigitalOut pin used as SD Card chip select
     * @param name The name used to access the virtual filesystem
     */
    SDCard(PinName, PinName, PinName, PinName);
    virtual ~SDCard() {};

    typedef enum {
        SDCARD_FAIL,
        SDCARD_V1,
        SDCARD_V2,
        SDCARD_V2HC
    } CARD_TYPE;

    virtual int disk_initialize();
    virtual int disk_write(const char *buffer, uint32_t block_number);
    virtual int disk_read(char *buffer, uint32_t block_number);
    virtual int disk_status();
    virtual int disk_sync();
    virtual uint32_t disk_sectors();
    virtual uint64_t disk_size();
    virtual uint32_t disk_blocksize();
    virtual bool disk_canDMA(void);

    CARD_TYPE card_type(void);

    bool busy();

protected:

    int _cmd(int cmd, uint32_t arg);
    int _cmdx(int cmd, uint32_t arg);
    int _cmd8();
    int _cmd58(uint32_t*);
    CARD_TYPE initialise_card();
    CARD_TYPE initialise_card_v1();
    CARD_TYPE initialise_card_v2();

    int _read(char *buffer, int length);
    int _write(const char *buffer, int length);

    uint32_t _sd_sectors();
    uint32_t _sectors;

    mbed::SPI _spi;
    GPIO _cs;

    volatile bool busyflag;

    CARD_TYPE cardtype;
};

#endif
