/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "UniversalAdapter.h"

#include "Kernel.h"
#include "checksumm.h"
#include "libs/Config.h"
#include "ConfigValue.h"
#include "libs/Pin.h"
#include "StreamOutputPool.h"

// config settings
#define panel_checksum             CHECKSUM("panel")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")
#define busy_pin_checksum          CHECKSUM("busy_pin")

// commands to Universal Adapter protocol version >= 0.97
#define GET_STATUS   1<<5
#define SET_CURSOR   2<<5
#define LCD_WRITE    3<<5
#define LCD_CLEAR    4<<5
#define SET_LEDS     5<<5
#define BUZZ         6<<5

#define READ_BUTTONS (GET_STATUS|0)
#define READ_ENCODER (GET_STATUS|1)
#define INIT_ADAPTER 0xFE
#define POLL         0xFF

// helper class to assert and deassert chip select
UniversalAdapter::SPIFrame::SPIFrame(UniversalAdapter *pu)
{
    this->u = pu;
    u->cs_pin->set(0);
}
UniversalAdapter::SPIFrame::~SPIFrame()
{
    u->cs_pin->set(1);
}

UniversalAdapter::UniversalAdapter()
{
    // configure the pins to use
    this->cs_pin = new Pin();
    this->cs_pin->from_string(THEKERNEL->config->value( panel_checksum, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();

    this->busy_pin = new Pin();
    this->busy_pin->from_string(THEKERNEL->config->value( panel_checksum, busy_pin_checksum)->by_default("nc")->as_string())->as_input();

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(panel_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName mosi;
    PinName miso;
    PinName sclk;
    if(spi_channel == 0) {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    } else if(spi_channel == 1) {
        mosi = P0_9; miso = P0_8; sclk = P0_7;
    } else {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    }

    this->spi = new mbed::SPI(mosi, miso, sclk);
    // chip select not selected
    this->cs_pin->set(1);

    int spi_frequency = THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(500000)->as_int();
    this->spi->frequency(spi_frequency);
    ledBits = 0;
}

UniversalAdapter::~UniversalAdapter()
{
    this->cs_pin->set(1);
    delete cs_pin;
    delete busy_pin;
    delete this->spi;
}

// void UniversalAdapter::on_refresh(bool now)
// {
//     SPIFrame sf(this); // asserts cs on entry and deasserts on exit
//     uint8_t b = sendReadCmd(GET_STATUS|2);
//     uint8_t cmd = SET_LEDS | 1;
//     writeSPI(cmd);
//     writeSPI(b);
// }

uint8_t UniversalAdapter::writeSPI(uint8_t b)
{
    uint8_t r = this->spi->write(b);
    wait_us(40); // need some delay here for arduino to catch up
    return r;
}

void UniversalAdapter::wait_until_ready()
{
    while(this->busy_pin->get() != 0) {
        // poll adapter for more room
        wait_ms(100);
        writeSPI(POLL);
    }
}

uint8_t UniversalAdapter::sendReadCmd(uint8_t cmd)
{
    writeSPI(cmd);
    return writeSPI(0);
}

uint8_t UniversalAdapter::readButtons()
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    uint8_t b = sendReadCmd(READ_BUTTONS);
    return b & ~BUTTON_PAUSE; // clear pause for now in case of noise
}

int UniversalAdapter::readEncoderDelta()
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    uint8_t e = sendReadCmd(READ_ENCODER);
    //if(e != 0) THEKERNEL->streams->printf("e: %02X\n", e);

    // this is actually a signed number +/-127, so convert to int
    int d = e < 128 ? e : -(256 - e);
    return d;
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void UniversalAdapter::buzz(long duration, uint16_t freq)
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    wait_until_ready();
    writeSPI(BUZZ);
}

void UniversalAdapter::write(const char *line, int len)
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    wait_until_ready();
    if(len > 31) {
        // this is the limit the UPA can handle in one frame (31 bytes)
        len = 31;
    }
    uint8_t cmd = LCD_WRITE | (len & 0x1F);
    writeSPI(cmd);
    for (int i = 0; i < len; ++i) {
        writeSPI(*line++);
    }
}

// Sets the indicator leds
void UniversalAdapter::setLed(int led, bool onoff)
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    if(onoff) {
        switch(led) {
            case LED_FAN_ON: ledBits    |= 1; break; // on
            case LED_HOTEND_ON: ledBits |= 2; break; // on
            case LED_BED_ON: ledBits    |= 4; break; // on
        }
    } else {
        switch(led) {
            case LED_FAN_ON: ledBits    &= ~1; break; // off
            case LED_HOTEND_ON: ledBits &= ~2; break; // off
            case LED_BED_ON: ledBits    &= ~4; break; // off
        }
    }
    uint8_t cmd = SET_LEDS | 1;
    wait_until_ready();
    writeSPI(cmd);
    writeSPI(ledBits);
}

void UniversalAdapter::clear()
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    wait_until_ready();
    writeSPI(LCD_CLEAR);
}

void UniversalAdapter::display()
{
    // it is always on
}

void UniversalAdapter::setCursor(uint8_t col, uint8_t row)
{
    SPIFrame sf(this); // asserts cs on entry and deasserts on exit
    wait_until_ready();
    uint8_t cmd = SET_CURSOR | 1;
    uint8_t rc = (row << 5) | (col & 0x1F);
    writeSPI(cmd);
    writeSPI(rc);
}

void UniversalAdapter::home()
{
    setCursor(0, 0);
}

void UniversalAdapter::init()
{
    {
        SPIFrame sf(this); // asserts cs on entry and deasserts on exit
        // send an init command
        writeSPI(INIT_ADAPTER);
    }
    // give adapter time to init
    wait_ms(100);
}
