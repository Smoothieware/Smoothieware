/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.

Much of this was copied from LiquidTWI2
  LiquidTWI2 High Performance i2c LCD driver for MCP23008 & MCP23017
  hacked by Sam C. Lin / http://www.lincomatic.com
  from
   LiquidTWI by Matt Falcon (FalconFour) / http://falconfour.com
   logic gleaned from Adafruit RGB LCD Shield library
*/

#ifndef VIKILCD_H
#define VIKILCD_H
#include "LcdBase.h"
#include "libs/Pin.h"
#include "mbed.h"

// VikiLcd specific settings

class VikiLCD : public LcdBase {
    public:
        VikiLCD();
        virtual ~VikiLCD();
        void home();
        void clear();
        void display();
        void setCursor(uint8_t col, uint8_t row);
        void init();
        void write(const char* line, int len);
        void on_refresh();

        // added viki commands
        void setLed(int led, bool onoff);

        uint8_t readButtons();
        int readEncoderDelta();

        // this is the number of clicks per detent
        int getEncoderResolution() { return 2; }

        void set_variant(int n) { isPanelolu2= (n==1); }

        void buzz(long,uint16_t);
        void noCursor();
        void cursor();
        void noBlink();
        void blink();
        void scrollDisplayLeft();
        void scrollDisplayRight();
        void leftToRight();
        void rightToLeft();
        void autoscroll();
        void noAutoscroll();
        void noDisplay();

    private:
        void send(uint8_t, uint8_t);
        void command(uint8_t value);

        void burstBits16(uint16_t);
        void burstBits8b(uint8_t);
        char displaymode;
        char displayfunction;
        char displaycontrol;
        char i2c_address;
        uint8_t _numlines,_currline;
        uint16_t _backlightBits; // only for MCP23017
        mbed::I2C* i2c;
        int i2c_frequency;
        Pin encoder_a_pin;
        Pin encoder_b_pin;
        Pin button_pause_pin;
        bool isPanelolu2;
};


#endif
