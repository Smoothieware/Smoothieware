/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "ReprapDiscountGLCD.h"

#include "Kernel.h"
#include "checksumm.h"
#include "libs/Config.h"
#include "rrdglcd/RrdGlcd.h"
#include "ConfigValue.h"

// config settings
#define panel_checksum             CHECKSUM("panel")
#define encoder_a_pin_checksum     CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum     CHECKSUM("encoder_b_pin")
#define click_button_pin_checksum  CHECKSUM("click_button_pin")
#define pause_button_pin_checksum  CHECKSUM("pause_button_pin")
#define back_button_pin_checksum   CHECKSUM("back_button_pin")
#define buzz_pin_checksum          CHECKSUM("buzz_pin")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")

ReprapDiscountGLCD::ReprapDiscountGLCD() {
    // configure the pins to use
    this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->click_pin.from_string(THEKERNEL->config->value( panel_checksum, click_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->pause_pin.from_string(THEKERNEL->config->value( panel_checksum, pause_button_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->back_pin.from_string(THEKERNEL->config->value( panel_checksum, back_button_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->buzz_pin.from_string(THEKERNEL->config->value( panel_checksum, buzz_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->spi_cs_pin.from_string(THEKERNEL->config->value( panel_checksum, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();

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

    this->glcd= new RrdGlcd(mosi, sclk, this->spi_cs_pin);

    int spi_frequency = THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(1000000)->as_number();
    this->glcd->setFrequency(spi_frequency);
}

ReprapDiscountGLCD::~ReprapDiscountGLCD() {
    delete this->glcd;
}

uint8_t ReprapDiscountGLCD::readButtons() {
    uint8_t state= 0;
    state |= (this->click_pin.get() ? BUTTON_SELECT : 0);
    // check the pause button
    if(this->pause_pin.connected() && this->pause_pin.get()) state |= BUTTON_PAUSE;
    if(this->back_pin.connected() && this->back_pin.get()) state |= BUTTON_LEFT;
    return state;
}

int ReprapDiscountGLCD::readEncoderDelta() {
    static const int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t old_AB = 0;
    old_AB <<= 2;                   //remember previous state
    old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state
    return  enc_states[(old_AB&0x0f)];
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void ReprapDiscountGLCD::buzz(long duration, uint16_t freq) {
    if(!this->buzz_pin.connected()) return;

    duration *=1000; //convert from ms to us
    long period = 1000000 / freq; // period in us
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        this->buzz_pin.set(1);
        wait_us(period / 2);
        this->buzz_pin.set(0);
        wait_us(period / 2);
        elapsed_time += (period);
    }
}

void ReprapDiscountGLCD::write(const char* line, int len) {
    this->glcd->displayString(this->row, this->col, line, len);
    this->col+=len;
}

void ReprapDiscountGLCD::home(){
    this->col= 0;
    this->row= 0;
}

void ReprapDiscountGLCD::clear(){
    this->glcd->clearScreen();
    this->col= 0;
    this->row= 0;
}

void ReprapDiscountGLCD::display() {
    // it is always on
}

void ReprapDiscountGLCD::setCursor(uint8_t col, uint8_t row){
    this->col= col;
    this->row= row;
}

void ReprapDiscountGLCD::init(){
    this->glcd->initDisplay();
}

// displays a selectable rectangle from the glyph
void ReprapDiscountGLCD::bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span, int x_offset, int y_offset) {
    if(x_offset == 0 && y_offset == 0 && span == 0) {
        // blt the whole thing
        this->glcd->renderGlyph(x, y, glyph, w, h);

    }else{
        // copy portion of glyph into g where x_offset is left byte aligned
        // Note currently the x_offset must be byte aligned
        int n= w/8; // bytes per line to copy
        if(w%8 != 0) n++; // round up to next byte
        uint8_t g[n*h];
        uint8_t *dst= g;
        const uint8_t *src= &glyph[y_offset*span + x_offset/8];
        for (int i = 0; i < h; ++i) {
            memcpy(dst, src, n);
            dst+=n;
            src+= span;
        }
        this->glcd->renderGlyph(x, y, g, w, h);
    }
}

void ReprapDiscountGLCD::on_refresh(bool now){
    static int refresh_counts = 0;
    refresh_counts++;
    // 10Hz refresh rate
    if(now || refresh_counts % 2 == 0 ) this->glcd->refresh();
}
