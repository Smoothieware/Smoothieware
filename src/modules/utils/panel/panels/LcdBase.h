#ifndef LCDBASE_H
#define LCDBASE_H

#include "stdint.h"

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

// specific LED assignments
#define LED_FAN_ON    1
#define LED_HOTEND_ON 2
#define LED_BED_ON    3

// Standard directional button bits
#define BUTTON_SELECT 0x01
#define BUTTON_RIGHT  0x02
#define BUTTON_DOWN   0x04
#define BUTTON_UP     0x08
#define BUTTON_LEFT   0x10
#define BUTTON_PAUSE  0x20
#define BUTTON_AUX1   0x40
#define BUTTON_AUX2   0x80

class Panel;

class LcdBase {
    public:
        LcdBase();
        virtual ~LcdBase();
        virtual void init()= 0;
        // only use this to display lines
        int printf(const char* format, ...);

        void setPanel(Panel* p) { panel= p; }

        // Required LCD functions
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
        virtual void setLed(int led, bool onoff){};
        virtual void setLedBrightness(int led, int val){};
        virtual void buzz(long,uint16_t){};
        virtual bool hasGraphics() { return false; }
        // on graphics panels, the input bitmap is in X windows XBM format but
        // with the bits in a byte reversed so bit7 is left most and bit0 is
        // right most. x_offset must by byte aligned if used
        virtual void bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span= 0, int x_offset=0, int y_offset=0){}
        // only used on certain panels
        virtual void on_refresh(bool now= false){};
        virtual void on_main_loop(){};
        // override this if the panel can handle more or less screen lines
        virtual uint16_t get_screen_lines() { return 4; }
        // used to set a variant for a panel (like viki vs panelolou2)
        virtual void set_variant(int n) {};

    protected:
        Panel* panel;
        virtual void write(const char* line, int len)= 0;

};

#endif // LCDBASE_H

