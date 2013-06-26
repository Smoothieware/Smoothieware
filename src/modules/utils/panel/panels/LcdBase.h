#ifndef LCDBASE_H
#define LCDBASE_H

#include "I2C.h" // mbed.h lib
#include "wait_api.h" // mbed.h lib

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"

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

// convenience LED defintions
#define LED_1  0x0001
#define LED_2  0x0002
#define LED_3  0x0004
#define LED_4  0x0008
#define LED_5  0x0010
#define LED_6  0x0020
#define LED_7  0x0040
#define LED_8  0x0080
#define LED_9  0x0100
#define LED_10 0x0200
#define LED_11 0x0400
#define LED_12 0x0800
#define LED_13 0x1000
#define LED_14 0x2000
#define LED_15 0x4000
#define LED_16 0x8000

// Standard directional button bits
#define BUTTON_SELECT 0x01
#define BUTTON_RIGHT  0x02
#define BUTTON_DOWN   0x04
#define BUTTON_UP     0x08
#define BUTTON_LEFT   0x10
#define BUTTON_AUX1   0x20
#define BUTTON_AUX2   0x40
#define BUTTON_AUX3   0x80

class Panel;

class LcdBase {
    public:
        LcdBase();
        virtual ~LcdBase();
        virtual void init()= 0;
        int printf(const std::string format, ...);

        void setPanel(Panel* p) { panel= p; }
        
        // Required LCD functions
        virtual void write(char value)= 0;
        virtual void home()= 0;
        virtual void clear()= 0;
        virtual void display()= 0;
        virtual void setCursor(uint8_t col, uint8_t row)= 0;

        // Returns button states including the encoder select button
        virtual uint8_t readButtons()= 0;

        // returns the current encoder position
        virtual int readEncoderDelta()= 0;
        
        // the number of encoder clicks per detent. this is divided into
        // accumulated clicks for control values so one detent is one
        // increment, this varies depending on encoder type usually 1,2 or 4
        virtual int getEncoderResolution()= 0;
                                            
        // optional
        virtual void setLed(uint16_t led, bool onoff){};
        virtual void setLedBrightness(uint16_t led, int val){};
        virtual void buzz(long,uint16_t){};

        // only used on certain panels
        virtual void on_refresh(){};
        virtual void on_main_loop(){};

    protected:
        Panel* panel;

};

#endif