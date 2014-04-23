/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef I2CLCD_H
#define I2CLCD_H

#include "LcdBase.h"

#include "I2C.h" // mbed.h lib
#include "wait_api.h" // mbed.h lib
#include "libs/Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

using namespace std;
#include <vector>
#include <string>
#include <cstdio>
#include <cstdarg>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 1<<2 // Enable bit
#define Rw 1<<1 // Read/Write bit
#define Rs 1<<0 // Register select bit

// config settings
#define panel_checksum             CHECKSUM("panel")
#define encoder_a_pin_checksum     CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum     CHECKSUM("encoder_b_pin")
#define up_button_pin_checksum     CHECKSUM("up_button_pin")
#define down_button_pin_checksum   CHECKSUM("down_button_pin")
#define click_button_pin_checksum  CHECKSUM("click_button_pin")

class I2CLCD : public LcdBase {
    public:
        I2CLCD() {
            // Default values
            this->i2c_address      = 0x27;
            this->backlightval     = 0x00;
            this->displaycontrol   = 0x00;
            this->displayfunction  = 0x00;
            this->displaymode      = 0x00;

            // I2C com
            this->i2c = new mbed::I2C(P0_27, P0_28);

            // configure the pins to use
            this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input()->pull_up();
            this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input()->pull_up();
            this->click_pin.from_string(THEKERNEL->config->value( panel_checksum, click_button_pin_checksum )->by_default("nc")->as_string())->as_input()->pull_up();
            this->up_pin.from_string(THEKERNEL->config->value( panel_checksum, up_button_pin_checksum)->by_default("nc")->as_string())->as_input()->pull_up();
            this->down_pin.from_string(THEKERNEL->config->value( panel_checksum, down_button_pin_checksum)->by_default("nc")->as_string())->as_input()->pull_up();

        }
        virtual ~I2CLCD() {
            delete this->i2c;
        }

        int getEncoderResolution() {
            return 1;
        }

        uint8_t readButtons() {
            uint8_t state= 0;
            state |= (this->click_pin.get() ? BUTTON_SELECT : 0);
            state |= (this->up_pin.get() ? BUTTON_UP : 0);
            state |= (this->down_pin.get() ? BUTTON_DOWN : 0);
            return state;
        }

        int readEncoderDelta() {
            static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
            static uint8_t old_AB = 0;
            old_AB <<= 2;                   //remember previous state
            old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state
            return  enc_states[(old_AB&0x0f)];
        }

        void expanderWrite(char data){
            this->i2c->start();
            this->i2c->write(this->i2c_address<<1);
            this->i2c->write((char)((char)data | (char)backlightval));
            this->i2c->stop();
        }

        void pulseEnable(char data){
            this->expanderWrite(data | En);      // En high
            wait_us(1);                          // enable pulse must be >450ns
            this->expanderWrite(data & ~En);     // En low
            wait_us(50);                         // commands need > 37us to settle
        }

        void write4bits(char value) {
            this->expanderWrite(value);
            this->pulseEnable(value);
        }

        void send(char value, char mode) {
            uint8_t highnib=value&0xf0;
            uint8_t lownib=(value<<4)&0xf0;
            this->write4bits((highnib)|mode);
            this->write4bits((lownib)|mode);
        }

        void command(char value) {
            this->send(value, 0);
        }

        void write(const char* line, int len) {
            for (int i = 0; i < len; ++i) {
                this->send(*line++, Rs);
            }
        }

        void home(){
            this->command(LCD_RETURNHOME);  // set cursor position to zero
            wait_us(2000);            // this command takes a long time!
        }

        void clear(){
            this->command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
            wait_us(2000);              // this command takes a long time!
        }

        void display() {
            this->displaycontrol |= LCD_DISPLAYON;
            this->command(LCD_DISPLAYCONTROL | this->displaycontrol);
        }

        void setCursor(uint8_t col, uint8_t row){
            int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
            if ( row > 4 ) {
                row = 4-1;    // we count rows starting w/0
            }
            this->command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
        }

        void init(){
            // Setup
            this->displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
            this->backlightval = LCD_NOBACKLIGHT;

            // Now we pull both RS and R/W low to begin commands
            wait_ms(50);
            this->expanderWrite(this->backlightval);
            wait_ms(1000);

            // we start in 8bit mode, try to set 4 bit mode
            for( char i=0;i<3;i++){
                this->write4bits(0x03 << 4);
                wait_us(4500);
            }

            // finally, set to 4-bit interface
            this->write4bits(0x02 << 4);

            // set # lines, font size, etc.
            this->command(LCD_FUNCTIONSET | this->displayfunction);

            // turn the display on with no cursor or blinking default
            this->displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
            this->display();

            // clear it off
            this->clear();

            // Initialize to default text direction (for roman languages)
            this->displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

            // set the entry mode
            this->command(LCD_ENTRYMODESET | displaymode);

            this->home();
            wait(0.1);

            this->backlightval=LCD_BACKLIGHT;
            expanderWrite(0);

        }

    private:
        char displaymode;
        char displayfunction;
        char displaycontrol;
        char i2c_address;
        char backlightval;

        mbed::I2C* i2c;

        Pin encoder_a_pin;
        Pin encoder_b_pin;
        Pin click_pin;
        Pin up_pin;
        Pin down_pin;
};


#endif
