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

#ifndef SMOOTHIEPANEL_H
#define SMOOTHIEPANEL_H
#include "LcdBase.h"
#include "libs/Pin.h"
#include "Button.h"
   
using namespace std;
#include <vector>
#include <string>
#include <cstdio>
#include <cstdarg>

// Smoothiepanel specific settings

// readButtons() will only return these bit values 
//#define ALL_BUTTON_BITS (BUTTON_AUX2|BUTTON_AUX1|BUTTON_SELECT)


#define PCA9505_ADDRESS 0x40
#define PCA9634_ADDRESS 0x50

// config settings for Smoothiepanel
#define panel_checksum             CHECKSUM("panel")
#define i2c_pins_checksum          CHECKSUM("i2c_pins")
#define i2c_address_checksum       CHECKSUM("i2c_address")
#define i2c_frequency_checksum     CHECKSUM("i2c_frequency")
#define i2c_interrupt_pin_checksum CHECKSUM("i2c_interrupt_pin")
#define encoder_a_pin_checksum     CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum     CHECKSUM("encoder_b_pin")

#define lcd_contrast_checksum      CHECKSUM("lcd_contrast")

class Smoothiepanel : public LcdBase {
    public:
        Smoothiepanel();
        ~Smoothiepanel();
        void home();
        void clear();
        void display();
        void setCursor(uint8_t col, uint8_t row);
        void init();
        void write(char value);

        // added viki commands
        void setBacklight(uint8_t status); 

        uint8_t readButtons();
        int readEncoderDelta();
        int getEncoderResolution() { return 2; }

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
/*
        void send(uint8_t, uint8_t);
*/
        void command(uint8_t value);
/*
        void burstBits16(uint16_t);
        void burstBits8b(uint8_t);
*/
        char displaymode;
        char displayfunction;
        char displaycontrol;
        char i2c_address;
        int i2c_frequency;
        int lcd_contrast;
        char backlightval;
        uint8_t _numlines,_currline;
//        uint16_t _backlightBits; // only for MCP23017
        mbed::I2C* i2c;

        Pin interrupt_pin;
        Pin encoder_a_pin;
        Pin encoder_b_pin;    
//        Button button_pause;
//        uint32_t on_pause_release(uint32_t dummy);
        bool paused;
};

#endif // SMOOTHIEPANEL_H
