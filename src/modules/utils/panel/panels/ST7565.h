/*
 * ST7565.h
 *
 *  Created on: 21-06-2013
 *      Author: Wulfnor
 */

#ifndef ST7565_H_
#define ST7565_H_

#include "LcdBase.h"
#include "mbed.h"
#include "libs/Pin.h"

class ST7565: public LcdBase {
public:
	ST7565(uint8_t v= 0);
	virtual ~ST7565();
	void home();
    void clear();
    void display();
    void setCursor(uint8_t col, uint8_t row);
	void init();
	void write_char(char value);
	void write(const char* line, int len);

	void on_refresh(bool now=false);
	//encoder which dosent exist :/
	uint8_t readButtons();
	int readEncoderDelta();
	int getEncoderResolution() { return is_viki2 ? 4 : 2; }
	uint16_t get_screen_lines() { return 8; }
	bool hasGraphics() { return true; }

	//added ST7565 commands
	void send_commands(const unsigned char* buf, size_t size);
	void send_data(const unsigned char* buf, size_t size);
	// set column and page number
	void set_xy(int x, int y);
	//send pic to whole screen
	void send_pic(const unsigned char* data);
	//drawing char
	int drawChar(int x, int y, unsigned char c, int color);
    // blit a glyph of w pixels wide and h pixels high to x, y. offset pixel position in glyph by x_offset, y_offset.
    // span is the width in bytes of the src bitmap
    // The glyph bytes will be 8 bits of X pixels, msbit->lsbit from top left to bottom right
    void bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span= 0, int x_offset=0, int y_offset=0);
    void renderGlyph(int x, int y, const uint8_t *g, int pixelWidth, int pixelHeight);
    void pixel(int x, int y, int colour);

    uint8_t getContrast() { return contrast; }
    void setContrast(uint8_t c);

    void buzz(long duration, uint16_t freq);
    void setLed(int led, bool onoff);

private:

    //buffer
	unsigned char *framebuffer;
	mbed::SPI* spi;
	Pin cs;
	Pin rst;
	Pin a0;
	Pin click_pin;
	Pin up_pin;
    Pin down_pin;
    Pin buzz_pin;
	Pin aux_pin;
    Pin encoder_a_pin;
    Pin encoder_b_pin;
    Pin red_led;
    Pin blue_led;

	// text cursor position
	uint8_t tx, ty;
    uint8_t contrast;
    struct {
        bool reversed:1;
        bool is_viki2:1;
        bool is_mini_viki2:1;
        bool use_pause:1;
        bool use_back:1;
    };
};

#endif /* ST7565_H_ */
