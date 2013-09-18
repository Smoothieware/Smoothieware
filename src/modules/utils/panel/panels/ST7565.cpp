/*
 * ST7565.cpp
 *
 *  Created on: 21-06-2013
 *      Author: Wulfnor
 */

#include "ST7565.h"
#include "ST7565/glcdfont.h"
#include "ahbmalloc.h"

#define CLAMP(x, low, high) { if ( (x) < (low) ) x = (low); if ( (x) > (high) ) x = (high); } while (0);
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

ST7565::ST7565() {
	//SPI com
    this->spi= new mbed::SPI(P0_18,P0_17,P0_15); //should be able to set those pins in config
    this->spi->frequency(THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(1000000)->as_number()); //4Mhz freq, can try go a little lower
    //chip select
    cs.from_string("0.16")->as_output(); //this also should be configurable
    cs.set(1);
    //lcd reset
    rst.from_string("2.8")->as_output(); //this also should be configurable
    rst.set(1);
    //a0
    a0.from_string("2.13")->as_output(); //this also should be configurable
    a0.set(1);

    this->click_pin.from_string(THEKERNEL->config->value( panel_checksum, click_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->up_pin.from_string(THEKERNEL->config->value( panel_checksum, up_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->down_pin.from_string(THEKERNEL->config->value( panel_checksum, down_button_pin_checksum )->by_default("nc")->as_string())->as_input();

    framebuffer= (uint8_t *)ahbmalloc(FB_SIZE, AHB_BANK_0); // grab some memoery from USB_RAM
    if(framebuffer == NULL) {
        THEKERNEL->streams->printf("Not enough memory available for frame buffer");
    }
}

ST7565::~ST7565() {
	//delete this->spi;
}

//send commands to lcd
void ST7565::send_commands(const unsigned char* buf, size_t size){
    cs.set(0);
    a0.set(0);
    while(size-- >0){
    	spi->write(*buf++);
    }
    cs.set(1);
}

//send data to lcd
void ST7565::send_data(const unsigned char* buf, size_t size){
    cs.set(0);
    a0.set(1);
    while(size-- >0){
    	spi->write(*buf++);
    }
    cs.set(1);
    a0.set(0);
}

//clearing screen
void ST7565::clear(){
    memset(framebuffer, 0, FB_SIZE);
    this->tx=0;
    this->ty=0;
}

void ST7565::send_pic(const unsigned char* data){
    for (int i=0; i<LCDPAGES; i++)
    {
    	set_xy(0, i);
        send_data(data + i*LCDWIDTH, LCDWIDTH);
    }
}

// set column and page number
void ST7565::set_xy(int x, int y)
{
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDPAGES-1);
    unsigned char cmd[3];
    cmd[0] = 0xb0 | (y & 0x07);
    cmd[1] = 0x10 | (x >> 4);
    cmd[2] = 0x00 | (x & 0x0f);
    send_commands(cmd, 3);
}

void ST7565::setCursor(uint8_t col, uint8_t row){
  this->tx=col*6;
  this->ty=row*8;
}

void ST7565::home(){
	this->tx=0;
	this->ty=0;
}

void ST7565::display(){
	///nothing
}

void ST7565::init(){
    const unsigned char init_seq[] = {
      0x40,    //Display start line 0
      0xa1,    //ADC reverse
      0xc0,    //Normal COM0...COM63
      0xa6,    //Display normal
      0xa2,    //Set Bias 1/9 (Duty 1/65)
      0x2f,    //Booster, Regulator and Follower On
      0xf8,    //Set internal Booster to 4x
      0x00,
      0x27,    //Contrast set
      0x81,
      0x09,    //contrast value
      0xac,    //No indicator
      0x00,
      0xaf,    //Display on
  };
  //rst.set(0);
  rst.set(1);
  send_commands(init_seq, sizeof(init_seq));
  clear();
}
int ST7565::drawChar(int x, int y, unsigned char c, int color){
	int retVal=-1;
	  if(c=='\n'){
	    this->ty+=8;
	    retVal= -tx;
	  }
	  if(c=='\r'){
		  retVal= -tx;
	  }
	  else{
	    for (uint8_t i =0; i<5; i++ ) {
	      if(color==0){
	        framebuffer[x + (y/8*128) ] = ~(glcd_font[(c*5)+i]<< y%8);
	        if(y+8<63){
	          framebuffer[x + ((y+8)/8*128) ] = ~(glcd_font[(c*5)+i] >>(8-(y%8)));
	        }
	      }
	      if(color==1){
	        framebuffer[x + ((y)/8*128) ] = glcd_font[(c*5)+i] <<(y%8);
	        if(y+8<63){
	        	framebuffer[x + ((y+8)/8*128) ] = glcd_font[(c*5)+i] >>(8-(y%8));
	        }
	      }
	      x++;
	    }
	    retVal= 6;
	    this->tx+=6;
	  }

	return retVal;
}

//write single char to screen
void ST7565::write_char(char value){
	drawChar(this->tx, this->ty,value,1);
}

void ST7565::write(const char* line, int len){
    for (int i = 0; i < len; ++i) {
    	write_char(line[i]);
    }
}
//refreshing screen

void ST7565::on_refresh(bool now){
    static int refresh_counts = 0;
    refresh_counts++;
    // 10Hz refresh rate
    if(now || refresh_counts % 2 == 0 ){
		send_pic(framebuffer);
	}
}

//reading button state
uint8_t ST7565::readButtons(void) {
    uint8_t state= 0;
    state |= (this->click_pin.get() ? BUTTON_SELECT : 0);
    state |= (this->up_pin.get() ? BUTTON_UP : 0);
    state |= (this->down_pin.get() ? BUTTON_DOWN : 0);
    return state;
}

int ST7565::readEncoderDelta() {
	return 0;
}

void ST7565::bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span, int x_offset, int y_offset) {
    if(x_offset == 0 && y_offset == 0 && span == 0) {
        // blt the whole thing
        renderGlyph(x, y, glyph, w, h);

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
        renderGlyph(x, y, g, w, h);
    }
}

void ST7565::renderGlyph(int x, int y, const uint8_t *g, int w, int h) {
    CLAMP(x, 0, LCDWIDTH-1);
    CLAMP(y, 0, LCDHEIGHT-1);
    CLAMP(w, 0, LCDWIDTH - x);
    CLAMP(h, 0, LCDHEIGHT - y);

    for(int i=0; i<w; i++){
    	for(int j=0; j<h; j++){
    	 pixel(x+i,y+j,g[(i/8)+ j*((w-1)/8 +1)] & (1<<(7-i%8)));
    	}
    }
}

void ST7565::pixel(int x, int y, int colour)
{
    int page = y / 8;
    unsigned char mask = 1<<(y%8);
    unsigned char *byte = &framebuffer[page*LCDWIDTH + x];
    if ( colour == 0 )
        *byte &= ~mask; // clear pixel
    else
        *byte |= mask; // set pixel
}
