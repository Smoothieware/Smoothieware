/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef RRDGLCD_H
#define RRDGLCD_H

#include "LcdBase.h"
#include "Pin.h"

class RrdGlcd;

class ReprapDiscountGLCD : public LcdBase {
    public:
        ReprapDiscountGLCD();
        virtual ~ReprapDiscountGLCD();

        int getEncoderResolution() { return 2; }
        bool hasGraphics() { return true; }
        uint16_t get_screen_lines() { return 8; }

        uint8_t readButtons();
        int readEncoderDelta();
        void write(const char* line, int len);
        void home();
        void clear();
        void display();
        void setCursor(uint8_t col, uint8_t row);
        void init();
        void buzz(long,uint16_t);
        // blit a glyph of w pixels wide and h pixels high to x, y. offset pixel position in glyph by x_offset, y_offset.
        // span is the width in bytes of the src bitmap
        // The glyph bytes will be 8 bits of X pixels, msbit->lsbit from top left to bottom right
        void bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span= 0, int x_offset=0, int y_offset=0);
        void on_refresh(bool now=false);

    private:
        RrdGlcd* glcd;
        uint8_t col;
        uint8_t row;

        Pin spi_cs_pin;
        Pin encoder_a_pin;
        Pin encoder_b_pin;
        Pin click_pin;
        Pin pause_pin;
        Pin back_pin;
        Pin buzz_pin;
};


#endif
