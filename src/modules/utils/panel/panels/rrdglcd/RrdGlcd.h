#ifndef __RRDGLCD_H
#define __RRDGLCD_H

/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * Based loosely on st7920.h from http://mbed.org/users/Bas/code/ST7920 and parts of the Arduino U8glib library.
 * Written by Jim Morris
 */

#include <mbed.h>
#include "libs/Kernel.h"
#include "libs/utils.h"
#include <libs/Pin.h>


class RrdGlcd {
public:
    /**
    *@brief Constructor, initializes the lcd on the respective pins.
    *@param mosi mbed pinname for mosi
    *@param sclk mbed name for sclk
    *@param cd Smoothie Pin for cs
    *@return none
    */
    RrdGlcd (int spi_channel, Pin cs);

    virtual ~RrdGlcd();

    void setFrequency(int f);

    void initDisplay(void);
    void clearScreen(void);
    void displayString(const char *ptr, int length);
    void refresh();
    void set_background(bool bg);
    void set_color(int c);
    void set_cursor(uint8_t col, uint8_t row);
    void set_cursorPX(int x, int y);
    void drawByte(int index, uint8_t mask, int color);   
     /**
    *@brief Fills the screen with the graphics described in a 1024-byte array
    *@
    *@param bitmap 128x64, bytes horizontal
    *@return none
    *
    */
    void fillGDRAM(const uint8_t *bitmap);

    // copy the bits in g, of X line size pixels, to x, y in frame buffer
    void renderGlyph(int x, int y, const uint8_t *g, int pixelWidth, int pixelHeight);
    // TODO: Implement new graphics functions for RRDGLCD
    void pixel(int x, int y, int color);
    void drawLine(int x0, int y0, int x1, int y1,int color);
    void drawHLine(int x, int y, int w, int color);
    void drawVLine(int x, int y, int h, int color);
    void drawBox(int x, int y, int w, int h, int color);
    // void setCursorPX(int x, int y);

private:
    Pin cs;
    mbed::SPI* spi;
    int drawChar(int x, int y, unsigned char c, int color, bool bg);
    uint8_t tx, ty;
    uint8_t text_color = 1;

    uint8_t *fb;
    struct {
        bool inited:1;
        bool dirty:1;
        bool text_background:1;
    };
};
#endif

