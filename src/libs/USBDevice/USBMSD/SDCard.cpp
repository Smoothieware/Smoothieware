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

/* Introduction
 * ------------
 * SD and MMC cards support a number of interfaces, but common to them all
 * is one based on SPI. This is the one I'm implmenting because it means
 * it is much more portable even though not so performant, and we already
 * have the mbed SPI Interface!
 *
 * The main reference I'm using is Chapter 7, "SPI Mode" of:
 *  http://www.sdcard.org/developers/tech/sdcard/pls/Simplified_Physical_Layer_Spec.pdf
 *
 * SPI Startup
 * -----------
 * The SD card powers up in SD mode. The SPI interface mode is selected by
 * asserting CS low and sending the reset command (CMD0). The card will
 * respond with a (R1) response.
 *
 * CMD8 is optionally sent to determine the voltage range supported, and
 * indirectly determine whether it is a version 1.x SD/non-SD card or
 * version 2.x. I'll just ignore this for now.
 *
 * ACMD41 is repeatedly issued to initialise the card, until "in idle"
 * (bit 0) of the R1 response goes to '0', indicating it is initialised.
 *
 * You should also indicate whether the host supports High Capicity cards,
 * and check whether the card is high capacity - i'll also ignore this
 *
 * SPI Protocol
 * ------------
 * The SD SPI protocol is based on transactions made up of 8-bit words, with
 * the host starting every bus transaction by asserting the CS signal low. The
 * card always responds to commands, data blocks and errors.
 *
 * The protocol supports a CRC, but by default it is off (except for the
 * first reset CMD0, where the CRC can just be pre-calculated, and CMD8)
 * I'll leave the CRC off I think!
 *
 * Standard capacity cards have variable data block sizes, whereas High
 * Capacity cards fix the size of data block to 512 bytes. I'll therefore
 * just always use the Standard Capacity cards with a block size of 512 bytes.
 * This is set with CMD16.
 *
 * You can read and write single blocks (CMD17, CMD25) or multiple blocks
 * (CMD18, CMD25). For simplicity, I'll just use single block accesses. When
 * the card gets a read command, it responds with a response token, and then
 * a data token or an error.
 *
 * SPI Command Format
 * ------------------
 * Commands are 6-bytes long, containing the command, 32-bit argument, and CRC.
 *
 * +---------------+------------+------------+-----------+----------+--------------+
 * | 01 | cmd[5:0] | arg[31:24] | arg[23:16] | arg[15:8] | arg[7:0] | crc[6:0] | 1 |
 * +---------------+------------+------------+-----------+----------+--------------+
 *
 * As I'm not using CRC, I can fix that byte to what is needed for CMD0 (0x95)
 *
 * All Application Specific commands shall be preceded with APP_CMD (CMD55).
 *
 * SPI Response Format
 * -------------------
 * The main response format (R1) is a status byte (normally zero). Key flags:
 *  idle - 1 if the card is in an idle state/initialising
 *  cmd  - 1 if an illegal command code was detected
 *
 *    +-------------------------------------------------+
 * R1 | 0 | arg | addr | seq | crc | cmd | erase | idle |
 *    +-------------------------------------------------+
 *
 * R1b is the same, except it is followed by a busy signal (zeros) until
 * the first non-zero byte when it is ready again.
 *
 * Data Response Token
 * -------------------
 * Every data block written to the card is acknowledged by a byte
 * response token
 *
 * +----------------------+
 * | xxx | 0 | status | 1 |
 * +----------------------+
 *              010 - OK!
 *              101 - CRC Error
 *              110 - Write Error
 *
 * Single Block Read and Write
 * ---------------------------
 *
 * Block transfers have a byte header, followed by the data, followed
 * by a 16-bit CRC. In our case, the data will always be 512 bytes.
 *
 * +------+---------+---------+- -  - -+---------+-----------+----------+
 * | 0xFE | data[0] | data[1] |        | data[n] | crc[15:8] | crc[7:0] |
 * +------+---------+---------+- -  - -+---------+-----------+----------+
 */

#include <stdio.h>
#include <stdlib.h>

#include "SDCard.h"

static const uint8_t OXFF = 0xFF;

#define SD_COMMAND_TIMEOUT 5000

SDCard::SDCard(PinName mosi, PinName miso, PinName sclk, PinName cs) :
  _spi(mosi, miso, sclk), _cs(cs) {
    _cs.output();
    _cs = 1;
    busyflag = false;
    _sectors = 0;
}

#define R1_IDLE_STATE           (1 << 0)
#define R1_ERASE_RESET          (1 << 1)
#define R1_ILLEGAL_COMMAND      (1 << 2)
#define R1_COM_CRC_ERROR        (1 << 3)
#define R1_ERASE_SEQUENCE_ERROR (1 << 4)
#define R1_ADDRESS_ERROR        (1 << 5)
#define R1_PARAMETER_ERROR      (1 << 6)

// Types
//  - v1.x Standard Capacity
//  - v2.x Standard Capacity
//  - v2.x High Capacity
//  - Not recognised as an SD Card

// #define SDCARD_FAIL 0
// #define SDCARD_V1   1
// #define SDCARD_V2   2
// #define SDCARD_V2HC 3

#define BUSY_FLAG_MULTIREAD          1
#define BUSY_FLAG_MULTIWRITE         2
#define BUSY_FLAG_ENDREAD            4
#define BUSY_FLAG_ENDWRITE           8
#define BUSY_FLAG_WAITNOTBUSY       (1<<31)

#define SDCMD_GO_IDLE_STATE          0
#define SDCMD_ALL_SEND_CID           2
#define SDCMD_SEND_RELATIVE_ADDR     3
#define SDCMD_SET_DSR                4
#define SDCMD_SELECT_CARD            7
#define SDCMD_SEND_IF_COND           8
#define SDCMD_SEND_CSD               9
#define SDCMD_SEND_CID              10
#define SDCMD_STOP_TRANSMISSION     12
#define SDCMD_SEND_STATUS           13
#define SDCMD_GO_INACTIVE_STATE     15
#define SDCMD_SET_BLOCKLEN          16
#define SDCMD_READ_SINGLE_BLOCK     17
#define SDCMD_READ_MULTIPLE_BLOCK   18
#define SDCMD_WRITE_BLOCK           24
#define SDCMD_WRITE_MULTIPLE_BLOCK  25
#define SDCMD_PROGRAM_CSD           27
#define SDCMD_SET_WRITE_PROT        28
#define SDCMD_CLR_WRITE_PROT        29
#define SDCMD_SEND_WRITE_PROT       30
#define SDCMD_ERASE_WR_BLOCK_START  32
#define SDCMD_ERASE_WR_BLK_END      33
#define SDCMD_ERASE                 38
#define SDCMD_LOCK_UNLOCK           42
#define SDCMD_APP_CMD               55
#define SDCMD_GEN_CMD               56

#define SD_ACMD_SET_BUS_WIDTH            6
#define SD_ACMD_SD_STATUS               13
#define SD_ACMD_SEND_NUM_WR_BLOCKS      22
#define SD_ACMD_SET_WR_BLK_ERASE_COUNT  23
#define SD_ACMD_SD_SEND_OP_COND         41
#define SD_ACMD_SET_CLR_CARD_DETECT     42
#define SD_ACMD_SEND_CSR                51

#define SD_CARD_HIGH_CAPACITY           (1UL<<30)

#define BLOCK2ADDR(block)   (((cardtype == SDCARD_V1) || (cardtype == SDCARD_V2))?(block << 9):((cardtype == SDCARD_V2HC)?(block):0))

SDCard::CARD_TYPE SDCard::initialise_card() {
    // Set to 25kHz for initialisation, and clock card with cs = 1
    _spi.frequency(25000);
    _cs = 1;

    for(int i=0; i<24; i++) {
        _spi.write(0xFF);
    }

    // send CMD0, should return with all zeros except IDLE STATE set (bit 0)
    if(_cmd(SDCMD_GO_IDLE_STATE, 0) != R1_IDLE_STATE) {
        fprintf(stderr, "No disk, or could not put SD card in to SPI idle state\n");
        return cardtype = SDCARD_FAIL;
    }

    // send CMD8 to determine whther it is ver 2.x
    int r = _cmd8();
    if(r == R1_IDLE_STATE) {
        return initialise_card_v2();
    } else if(r == (R1_IDLE_STATE | R1_ILLEGAL_COMMAND)) {
        return initialise_card_v1();
    } else {
        fprintf(stderr, "Not in idle state after sending CMD8 (not an SD card?)\n");
        return cardtype = SDCARD_FAIL;
    }
}

SDCard::CARD_TYPE SDCard::initialise_card_v1() {
    for(int i=0; i<SD_COMMAND_TIMEOUT; i++) {
        _cmd(SDCMD_APP_CMD, 0);
        if(_cmd(SD_ACMD_SD_SEND_OP_COND, 0) == 0) {
            return cardtype = SDCARD_V1;
        }
    }

    fprintf(stderr, "Timeout waiting for v1.x card\n");
    return SDCARD_FAIL;
}

SDCard::CARD_TYPE SDCard::initialise_card_v2() {

    for(int i=0; i<SD_COMMAND_TIMEOUT; i++) {
        _cmd(SDCMD_APP_CMD, 0);
        if(_cmd(SD_ACMD_SD_SEND_OP_COND, SD_CARD_HIGH_CAPACITY) == 0) {
            uint32_t ocr;
            _cmd58(&ocr);
            if (ocr & SD_CARD_HIGH_CAPACITY)
                return cardtype = SDCARD_V2HC;
            else
                return cardtype = SDCARD_V2;
        }
    }

    fprintf(stderr, "Timeout waiting for v2.x card\n");
    return cardtype = SDCARD_FAIL;
}

int SDCard::disk_initialize()
{
    busyflag = true;

    _sectors = 0;

    CARD_TYPE i = initialise_card();

    if (i == SDCARD_FAIL) {
        busyflag = false;
        return 1;
    }

    _sectors = _sd_sectors();

    // Set block length to 512 (CMD16)
    if(_cmd(SDCMD_SET_BLOCKLEN, 512) != 0) {
        fprintf(stderr, "Set 512-byte block timed out\n");
        busyflag = false;
        return 1;
    }

    _spi.frequency(2500000); // Set to 2.5MHz for data transfer

    busyflag = false;

    return 0;
}

int SDCard::disk_write(const char *buffer, uint32_t block_number)
{
    if (busyflag)
        return 0;

    busyflag = true;

    if (cardtype == SDCARD_FAIL)
        return -1;
    // set write address for single block (CMD24)
    if(_cmd(SDCMD_WRITE_BLOCK, BLOCK2ADDR(block_number)) != 0) {
        return 1;
    }

    // send the data block
    _write(buffer, 512);

    busyflag = false;

    return 0;
}

int SDCard::disk_read(char *buffer, uint32_t block_number)
{
    if (busyflag)
        return 0;

    busyflag = true;

    if (cardtype == SDCARD_FAIL)
        return -1;
    // set read address for single block (CMD17)
    if(_cmd(SDCMD_READ_SINGLE_BLOCK, BLOCK2ADDR(block_number)) != 0) {
        return 1;
    }

    // receive the data
    _read(buffer, 512);

    busyflag = false;

    return 0;
}

int SDCard::disk_status() { return (_sectors > 0)?0:1; }
int SDCard::disk_sync() {
    // TODO: wait for DMA, wait for card not busy
    return 0;
}
uint32_t SDCard::disk_sectors() { return _sectors; }
uint64_t SDCard::disk_size() { return ((uint64_t) _sectors) << 9; }
uint32_t SDCard::disk_blocksize() { return (1<<9); }
bool SDCard::disk_canDMA() { return false; }

SDCard::CARD_TYPE SDCard::card_type()
{
    return cardtype;
}

// PRIVATE FUNCTIONS

int SDCard::_cmd(int cmd, uint32_t arg) {
    _cs = 0;

    // send a command
    _spi.write(0x40 | cmd);
    _spi.write(arg >> 24);
    _spi.write(arg >> 16);
    _spi.write(arg >> 8);
    _spi.write(arg >> 0);
    _spi.write(0x95);

    // wait for the repsonse (response[7] == 0)
    for(int i=0; i<SD_COMMAND_TIMEOUT; i++) {
        int response = _spi.write(0xFF);
        if(!(response & 0x80)) {
            _cs = 1;
            _spi.write(0xFF);
            return response;
        }
    }
    _cs = 1;
    _spi.write(0xFF);
    return -1; // timeout
}
int SDCard::_cmdx(int cmd, uint32_t arg) {
    _cs = 0;

    // send a command
    _spi.write(0x40 | cmd);
    _spi.write(arg >> 24);
    _spi.write(arg >> 16);
    _spi.write(arg >> 8);
    _spi.write(arg >> 0);
    _spi.write(0x95);

    // wait for the repsonse (response[7] == 0)
    for(int i=0; i<SD_COMMAND_TIMEOUT; i++) {
        int response = _spi.write(0xFF);
        if(!(response & 0x80)) {
            return response;
        }
    }
    _cs = 1;
    _spi.write(0xFF);
    return -1; // timeout
}


int SDCard::_cmd58(uint32_t *ocr) {
    _cs = 0;
    int arg = 0;

    // send a command
    _spi.write(0x40 | 58);
    _spi.write(arg >> 24);
    _spi.write(arg >> 16);
    _spi.write(arg >> 8);
    _spi.write(arg >> 0);
    _spi.write(0x95);

    // wait for the repsonse (response[7] == 0)
    for(int i=0; i<SD_COMMAND_TIMEOUT; i++) {
        int response = _spi.write(0xFF);
        if(!(response & 0x80)) {
            *ocr = _spi.write(0xFF) << 24;
            *ocr |= _spi.write(0xFF) << 16;
            *ocr |= _spi.write(0xFF) << 8;
            *ocr |= _spi.write(0xFF) << 0;
//            printf("OCR = 0x%08X\n", ocr);
            _cs = 1;
            _spi.write(0xFF);
            return response;
        }
    }
    _cs = 1;
    _spi.write(0xFF);
    return -1; // timeout
}

int SDCard::_cmd8() {
    _cs = 0;

    // send a command
    _spi.write(0x40 | SDCMD_SEND_IF_COND); // CMD8
    _spi.write(0x00);     // reserved
    _spi.write(0x00);     // reserved
    _spi.write(0x01);     // 3.3v
    _spi.write(0xAA);     // check pattern
    _spi.write(0x87);     // crc

    // wait for the repsonse (response[7] == 0)
    for(int i=0; i<SD_COMMAND_TIMEOUT * 1000; i++) {
        char response[5];
        response[0] = _spi.write(0xFF);
        if(!(response[0] & 0x80)) {
                for(int j=1; j<5; j++) {
                    response[i] = _spi.write(0xFF);
                }
                _cs = 1;
                _spi.write(0xFF);
                return response[0];
        }
    }
    _cs = 1;
    _spi.write(0xFF);
    return -1; // timeout
}

int SDCard::_read(char *buffer, int length) {
    _cs = 0;

    // read until start byte (0xFF)
    while(_spi.write(0xFF) != 0xFE);
//     uint8_t r;
//     while((r = _spi.write(0xFF)) != 0xFE)
//     {
//         iprintf("0x%02X ", r);
//         for (volatile uint32_t j = 262144; j; j--);
//     }
//
//     iprintf("Got start byte, reading data\n");

    // read data
    for(int i=0; i<length; i++) {
        buffer[i] = _spi.write(0xFF);
    }
    _spi.write(0xFF); // checksum
    _spi.write(0xFF);

    _cs = 1;
    _spi.write(0xFF);
    return 0;
}

int SDCard::_write(const char *buffer, int length) {
    _cs = 0;

    // indicate start of block
    _spi.write(0xFE);

    // write the data
    for(int i=0; i<length; i++) {
        _spi.write(buffer[i]);
    }

    // write the checksum
    _spi.write(0xFF);
    _spi.write(0xFF);

    // check the repsonse token
    if((_spi.write(0xFF) & 0x1F) != 0x05) {
        _cs = 1;
        _spi.write(0xFF);
        return 1;
    }

    // wait for write to finish
    while(_spi.write(0xFF) == 0);

    _cs = 1;
    _spi.write(0xFF);
    return 0;
}

static int ext_bits(char *data, int msb, int lsb) {
    int bits = 0;
    int size = 1 + msb - lsb;
    for(int i=0; i<size; i++) {
        int position = lsb + i;
        int byte = 15 - (position >> 3);
        int bit = position & 0x7;
        int value = (data[byte] >> bit) & 1;
        bits |= value << i;
    }
    return bits;
}

uint32_t SDCard::_sd_sectors() {

    // CMD9, Response R2 (R1 byte + 16-byte block read)
    if(_cmdx(SDCMD_SEND_CSD, 0) != 0) {
        fprintf(stderr, "Didn't get a response from the disk\n");
        return 0;
    }

    char csd[16];
    if(_read(csd, 16) != 0) {
        fprintf(stderr, "Couldn't read csd response from disk\n");
        return 0;
    }

    // csd_structure : csd[127:126]
    // c_size        : csd[73:62]
    // c_size_mult   : csd[49:47]
    // read_bl_len   : csd[83:80] - the *maximum* read block length

    int csd_structure = ext_bits(csd, 127, 126);

    if (csd_structure == 0)
    {
        if (cardtype == SDCARD_V2HC)
        {
            fprintf(stderr, "SDHC card with regular SD descriptor!\n");
            return 0;
        }
        uint32_t c_size = ext_bits(csd, 73, 62);
        uint32_t c_size_mult = ext_bits(csd, 49, 47);
        uint32_t read_bl_len = ext_bits(csd, 83, 80);

        uint32_t block_len = 1 << read_bl_len;
        uint32_t mult = 1 << (c_size_mult + 2);
        uint32_t blocknr = (c_size + 1) * mult;

        if (block_len >= 512)
            return blocknr * (block_len >> 9);
        else
            return (blocknr * block_len) >> 9;
    }
    else if (csd_structure == 1)
    {
        if (cardtype != SDCARD_V2HC)
        {
            fprintf(stderr, "SD V1 or V2 card with SDHC descriptor!\n");
            return 0;
        }
        uint32_t c_size = ext_bits(csd, 69, 48);
        uint32_t blocknr = (c_size + 1) * 1024;

        return blocknr;
    }
    fprintf(stderr, "This disk tastes funny! (%d) I only know about type 0 or 1 CSD structures\n", csd_structure);
    return 0;
}

bool SDCard::busy()
{
    return busyflag;
}
