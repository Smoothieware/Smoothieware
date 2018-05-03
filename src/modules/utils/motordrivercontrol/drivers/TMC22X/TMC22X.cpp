/*
 Highly modifed from....

 TMC26X.cpp - - TMC26X Stepper library for Wiring/Arduino

 based on the stepper library by Tom Igoe, et. al.

 Copyright (c) 2011, Interactive Matter, Marcus Nowotny

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 */

#include "TMC22X.h"
#include "mbed.h"
#include "StreamOutput.h"
#include "Kernel.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"
#include "StepTicker.h"

#define motor_driver_control_checksum  CHECKSUM("motor_driver_control")
#define sense_resistor_checksum        CHECKSUM("sense_resistor")

#define READ        0x00
#define WRITE       0x80
#define SYNC        0x05
#define SLAVEADDR   0x00


#define DEFAULT_DATA 0x00000000

//debuging output
//#define DEBUG

/*
 * Constructor
 */
TMC22X::TMC22X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> uart, char d) : uart(uart), designator(d)
{
    //we are not started yet
    //started = false;
    //by default cool step is not enabled
    //cool_step_enabled = false;
    //error_reported.reset();
}

/*
 * configure the stepper driver
 * just must be called.
 */
void TMC22X::init(uint16_t cs)
{
    // read chip specific config entries
    this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(50)->as_number(); // in milliohms

    //set the initial values
    send2208(0x80,0x00000040);
    send2208(0x6F,0x00000000);

    started = true;
}

//calculates CRC checksum and stores in last byte of message
void calc_crc(uint8_t *buf, int cnt)
{
    uint8_t *crc = buf + cnt -1; // CRC located in last byte of message
    uint8_t currentByte;

    *crc = 0;
    for (int i = 0; i < cnt-1; i++) {  // Execute for all bytes of a message
        currentByte = buf[i];          // Retrieve a byte to be sent from Array
        for (int j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) {   // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            } else {
                *crc = (*crc << 1);
            }
            //crc &= 0xff;
            currentByte = currentByte >> 1;
        }   // for CRC bit
    }       // for message byte
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status
 * sends 20bits, the last 20 bits of the 24bits is taken as the command
 */
bool TMC22X::send2208(uint8_t reg, uint32_t datagram)
{
    uint8_t rbuf[8];
    if(reg & WRITE) {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(datagram >> 24), (uint8_t)(datagram >> 16), (uint8_t)(datagram >> 8), (uint8_t)(datagram >> 0), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 8);

        //write/read the values
        uart(buf, 8, rbuf);
        THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    } else {
        uint8_t buf[] {(uint8_t)(SYNC), (uint8_t)(SLAVEADDR), (uint8_t)(reg), (uint8_t)(0x00)};

        //calculate checksum
        calc_crc(buf, 4);

        //write/read the values
        uart(buf, 4, rbuf);

        THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X \n", buf[0], buf[1], buf[2], buf[3], rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7]);
    }
    return true;
}
