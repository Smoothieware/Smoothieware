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
#include "mbed.h"


// Smoothiepanel specific settings

// readButtons() will only return these bit values
//#define ALL_BUTTON_BITS (BUTTON_AUX2|BUTTON_AUX1|BUTTON_SELECT)



// config settings for Smoothiepanel
#define panel_checksum             CHECKSUM("panel")
#define spi_pins_checksum          CHECKSUM("spi_pins")
#define spi_ssel_pin_checksum      CHECKSUM("spi_ssel_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")

#define lcd_type_checksum          CHECKSUM("lcd_type")
#define lcd_contrast_checksum      CHECKSUM("lcd_contrast")
#define lcd_led_checksum           CHECKSUM("lcd_led")
#define lcd_led_red_checksum       CHECKSUM("lcd_led_red")
#define lcd_led_green_checksum     CHECKSUM("lcd_led_green")
#define lcd_led_blue_checksum      CHECKSUM("lcd_led_blue")

#define wing1_checksum             CHECKSUM("wing1")
#define wing2_checksum             CHECKSUM("wing2")
#define button_wing_checksum       CHECKSUM("button_wing")
#define encoder_wing_checksum      CHECKSUM("encoder_wing")

//#define encoder_a_pin_checksum     CHECKSUM("encoder_a_pin")
//#define encoder_b_pin_checksum     CHECKSUM("encoder_b_pin")
#define encoder_led_red_checksum   CHECKSUM("encoder_red_red")
#define encoder_led_green_checksum   CHECKSUM("encoder_green_red")
#define encoder_led_blue_checksum   CHECKSUM("encoder_blue_red")

#define play_led_brightness_checksum CHECKSUM("play_led_brightness")
#define back_led_brightness_checksum CHECKSUM("back_led_brightness")

class Smoothiepanel : public LcdBase {
    public:
        Smoothiepanel();
        ~Smoothiepanel();

        void home();
        void clear();
        void display();
        void setCursor(uint8_t col, uint8_t row);
        void init();
        int send(int c);
        void write(const char* line, int len);

        void setBacklight(uint8_t status);
        void setBacklightColor(uint8_t r, uint8_t g, uint8_t b);
        void setBacklightByHue(int h);
        void setEncoderLED(uint8_t r, uint8_t g, uint8_t b);
        void setEncoderByHue(int h);
        void setPlayLED(uint8_t v);
        void setBackLED(uint8_t v);

        uint8_t readButtons();
        int readEncoderDelta();
        int getEncoderResolution() { return 4; }
        bool hasExternalEncoder() { return true; }

        void setLed(int led, bool on);
        void setLedBrightness(int led, int val);
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
        uint16_t wing1_type;
        uint16_t wing2_type;
        char displaymode;
        char displayfunction;
        char displaycontrol;
//        char i2c_address;
//        int i2c_frequency;
        int lcd_contrast;
//        int encoder_hue;
        char backlight_red;
        char backlight_green;
        char backlight_blue;
        char backlightval;
        char playledval;
        char backledval;
        uint8_t _numlines,_currline;
//        uint16_t _backlightBits; // only for MCP23017
//        mbed::I2C* i2c;
        mbed::SPI* spi;

        Pin spi_ssel_pin;
//        Pin interrupt_pin;
//        Pin encoder_a_pin;
//        Pin encoder_b_pin;
//        bool encoder_changed;
};

#endif // SMOOTHIEPANEL_H
