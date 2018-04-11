/*
 Modified from TMC26X.cpp

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

#include "TMC21X.h"
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

/*
 * Constructor
 */
TMC21X::TMC21X(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char d) : spi(spi), designator(d)
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
void TMC21X::init(uint16_t cs)
{
    //set the initial values
//    send2130(0x00,0x00000000);
//    send2130(0x80,0x00000000);
//    send2130(0xA0,0x00000000);
//    send2130(0xC0,0x00000000);
//    send2130(0xE0,0x00000000);

    //Reference
//    this->send2130(0x80,0x00000001UL); //voltage on AIN is current reference
//    this->send2130(0x90,0x00001010UL); //IHOLD=0x10, IRUN=0x10
//    this->send2130(0xEC,0x00008008UL); //native 256 microsteps, MRES=0, TBL=1=24, TOFF=8

    //Initialization example
//    this->send2130(0xEC,0x000100C3ul);    //CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
//    this->send2130(0x90,0x00061F0Aul);    //IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
//    this->send2130(0x91,0x0000000Aul);    //TPOWERDOWN=10: Delay before power down in stand still
//    this->send2130(0x80,0x00000004ul);    //EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
//    this->send2130(0x93,0x000001F4ul);    //TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
//    this->send2130(0xF0,0x000401C8ul);    //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1

    //Other example
    send2130(0x6F,0x00000000);
    send2130(0x6F,0x00000000);
    send2130(0xEC,0x00ABCDEF);
    send2130(0xEC,0x00123456);

    //All read status
//    send2130(0x00,0x00000000);
//    send2130(0x01,0x00000000);
//    send2130(0x04,0x00000000);
//    send2130(0x12,0x00000000);
//    send2130(0x2D,0x00000000);
//    send2130(0x6A,0x00000000);
//    send2130(0x6B,0x00000000);
//    send2130(0x6C,0x00000000);
//    send2130(0x6F,0x00000000);
//    send2130(0x71,0x00000000);
//    send2130(0x73,0x00000000);
//    send2130(0x6F,0x00000000);
}

/*
 * send register settings to the stepper driver via SPI
 * returns the current status 40 bit datagram, first 8 bits is the status, the last 32 bits are the register contents
 */
void TMC21X::send2130(uint8_t reg, uint32_t datagram) //TODO Converted, needs testing
{
    uint8_t buf[5];

    //Note: SPI write first to last //TODO check that this is actually working as may have just swapped bytes and not the bit order
    buf[0] = (uint8_t)(reg);
    buf[1] = (uint8_t)(datagram >> 24);
    buf[2] = (uint8_t)(datagram >> 16);
    buf[3] = (uint8_t)(datagram >> 8);
    buf[4] = (uint8_t)(datagram >> 0);

    uint8_t rbuf[5];

    //write/read the values
    spi(buf, 5, rbuf);

    //print sent and received bytes
    THEKERNEL->streams->printf("sent: %02X, %02X, %02X, %02X, %02X received: %02X, %02X, %02X, %02X, %02X \n", buf[4], buf[3], buf[2], buf[1], buf[0], rbuf[4], rbuf[3], rbuf[2], rbuf[1], rbuf[0]);
}
