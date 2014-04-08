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

// config settings
#define panel_checksum             CHECKSUM("panel")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")

// commands to Universal Adapter
#define READ_BUTTONS 1<<5
#define READ_ENCODER 2<<5
#define LCD_WRITE    3<<5
#define LCD_CLEAR    4<<5
#define SET_LEDS     5<<5
#define BUZZ         6<<5
#define INIT         7<<5

UniversalAdapter::UniversalAdapter() {
    // configure the pins to use
    this->cs_pin= new Pin();
    this->cs_pin->from_string(THEKERNEL->config->value( panel_checksum, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(panel_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName mosi;
    PinName miso;
    PinName sclk;
    if(spi_channel == 0){
        mosi= P0_18; miso= P0_17; sclk= P0_15;
    }else if(spi_channel == 1){
        mosi= P0_9; miso= P0_8; sclk= P0_7;
    }else{
        mosi= P0_18; miso= P0_17; sclk= P0_15;
    }

    this->spi= new mbed::SPI(mosi, miso, sclk);
    // chip select
    this->cs_pin->set(1);

    int spi_frequency = THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(100000)->as_number();
    this->spi->frequency(spi_frequency);
    ledBits= 0;
}

UniversalAdapter::~UniversalAdapter() {
    this->cs_pin->set(1);
    delete cs_pin;
    delete this->spi;
}

uint8_t UniversalAdapter::sendReadCmd(uint8_t cmd) {
    this->spi->write(cmd);
    wait_us(50);
    uint8_t n= this->spi->write(0);
    wait_us(50);
    return n;
}
uint8_t UniversalAdapter::readButtons() {
    return sendReadCmd(READ_BUTTONS);
}

int UniversalAdapter::readEncoderDelta() {
    int e= sendReadCmd(READ_ENCODER);
    // hack around we seem to be getting absolute values
    if(e > 0 && e < 128) return 1;
    else if(e > 128) return -1;
    return 0;
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void UniversalAdapter::buzz(long duration, uint16_t freq) {
    this->spi->write(BUZZ);
    wait_ms(100);
}

void UniversalAdapter::write(const char* line, int len) {
    uint8_t cmd= LCD_WRITE | ((len+1)&0x1F);
    uint8_t rc= (this->row << 5) | (this->col&0x1F);
    this->spi->write(cmd);
    this->spi->write(rc);
    for (int i = 0; i < len; ++i) {
        this->spi->write(*line++);
    }
    this->col+=len;
    wait_us(len*2000);
}

// Sets the indicator leds
void UniversalAdapter::setLed(int led, bool onoff) {
    if(onoff) {
        switch(led) {
            case LED_FAN_ON: ledBits    |= 1; break; // on
            case LED_HOTEND_ON: ledBits |= 2; break; // on
            case LED_BED_ON: ledBits    |= 4; break; // on
        }
    }else{
        switch(led) {
            case LED_FAN_ON: ledBits    &= ~1; break; // off
            case LED_HOTEND_ON: ledBits &= ~2; break; // off
            case LED_BED_ON: ledBits    &= ~4; break; // off
        }
    }
    uint8_t cmd= SET_LEDS | 1;
    this->spi->write(cmd);
    this->spi->write(ledBits);
    wait_ms(1);
}

void UniversalAdapter::home(){
    this->col= 0;
    this->row= 0;
}

void UniversalAdapter::clear(){
    this->spi->write(LCD_CLEAR);
    this->col= 0;
    this->row= 0;
    wait_ms(1);
}

void UniversalAdapter::display() {
    // it is always on
}

void UniversalAdapter::setCursor(uint8_t col, uint8_t row){
    this->col= col;
    this->row= row;
}

void UniversalAdapter::init(){
    // send an init toggle CS
    this->cs_pin->set(1);
    wait_ms(5);
    this->cs_pin->set(0);
    wait_ms(5);
}
