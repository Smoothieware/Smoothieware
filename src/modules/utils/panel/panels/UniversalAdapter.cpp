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

UniversalAdapter::UniversalAdapter() {
    // configure the pins to use
    this->cs_pin= new Pin();
    this->cs_pin->from_string(THEKERNEL->config->value( panel_checksum, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(panel_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName mosi;
    PinName sclk;
    if(spi_channel == 0){
        mosi= P0_18; sclk= P0_15;
    }else if(spi_channel == 1){
        mosi= P0_9; sclk= P0_7;
    }else{
        mosi= P0_18; sclk= P0_15;
    }

    this->spi= new mbed::SPI(mosi, NC, sclk);
    //chip select
    this->cs_pin->set(0);

    int spi_frequency = THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(1000000)->as_number();
    this->spi->frequency(spi_frequency);
}

UniversalAdapter::~UniversalAdapter() {
    this->cs_pin->set(1);
    delete cs_pin;
    delete this->spi;
}

uint8_t UniversalAdapter::readButtons() {
    this->spi->write(READ_BUTTONS);
    uint8_t b= this->spi->write(0);
    return b;
}

int UniversalAdapter::readEncoderDelta() {
    this->spi->write(READ_ENCODER);
    int e= this->spi->write(0);
    return e;
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void UniversalAdapter::buzz(long duration, uint16_t freq) {
    this->spi->write(BUZZ);
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
}

void UniversalAdapter::home(){
    this->col= 0;
    this->row= 0;
}

void UniversalAdapter::clear(){
    this->spi->write(LCD_CLEAR);
    this->col= 0;
    this->row= 0;
}

void UniversalAdapter::display() {
    // it is always on
}

void UniversalAdapter::setCursor(uint8_t col, uint8_t row){
    this->col= col;
    this->row= row;
}

void UniversalAdapter::init(){
}
