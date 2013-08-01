/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#include "ReprapDiscountGLCD.h"

ReprapDiscountGLCD::ReprapDiscountGLCD() {
    // configure the pins to use
    this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->click_pin.from_string(THEKERNEL->config->value( panel_checksum, click_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->pause_pin.from_string(THEKERNEL->config->value( panel_checksum, pause_button_pin_checksum)->by_default("nc")->as_string())->as_input();
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
    return state;
}

int ReprapDiscountGLCD::readEncoderDelta() { 
    static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
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
    for (int i = 0; i < len; ++i) {
        this->glcd->displayString(this->row, this->col, line, len);
    }
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

void ReprapDiscountGLCD::bltGlyph(int x, int y, int w, int h, const uint8_t *glyph) {
    // TODO
}

void ReprapDiscountGLCD::on_refresh(bool now){
    static int refresh_counts = 0;
    refresh_counts++;
    // 5Hz refresh rate
    if(now || refresh_counts % 4 == 0 ) this->glcd->refresh();
}