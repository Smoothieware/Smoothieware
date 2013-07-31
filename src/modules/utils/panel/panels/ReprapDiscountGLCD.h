/*  
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef RRDGLCD_H
#define RRDGLCD_H

#include "LcdBase.h"

#include "libs/Config.h"
#include "rrdglcd/RrdGlcd.h"

// config settings
#define panel_checksum             CHECKSUM("panel")
#define encoder_a_pin_checksum     CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum     CHECKSUM("encoder_b_pin")
#define click_button_pin_checksum  CHECKSUM("click_button_pin")
#define pause_button_pin_checksum  CHECKSUM("pause_button_pin")
#define buzz_pin_checksum          CHECKSUM("buzz_pin")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")

class ReprapDiscountGLCD : public LcdBase {
    public:
        ReprapDiscountGLCD();
        virtual ~ReprapDiscountGLCD();

        int getEncoderResolution() { return 2; }
        bool hasGraphics() { return false; }
        uint16_t get_screen_lines() { return 8; }

        uint8_t readButtons();
        int readEncoderDelta();
        void write(char value);
        void writeDone();
        void home();
        void clear();
        void display();
        void setCursor(uint8_t col, uint8_t row);
        void init();
        void buzz(long,uint16_t);
        // blit a glyph of w pixels wide and h pixels high to x,y. The glyph will be zero left padded bytes from top left to bottom right
        void bltGlyph(int x, int y, int w, int h, const uint8_t *glyph);
        void on_refresh();

    private:
        RrdGlcd* glcd;
        uint8_t col;
        uint8_t row;

        Pin spi_cs_pin;
        Pin encoder_a_pin;
        Pin encoder_b_pin;    
        Pin click_pin;
        Pin pause_pin;
        Pin buzz_pin;
};


#endif
