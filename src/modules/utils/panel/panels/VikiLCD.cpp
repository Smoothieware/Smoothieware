/*  
This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>. 
*/
#include "VikiLCD.h"

// if this is defined we use the R/W poll mode instead of fixed delays
// However at the slower I2C frequency required for Viki long cables it is slower than fixed delay
// taken from LiquidCrystalFast.cpp and implemented for Viki LCD here by Jim Morris
//#define USE_FASTMODE

//MCP23017 - Adafruit RGB LCD Shield and VikiLCD
// bit pattern for the burstBits function is
//
//  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017 
//  RS RW EN D4 D5 D6 D7 LB LG LR BZ B4 B3 B2 B1 B0 
//  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0  
#define M17_BIT_RS 0x8000
#define M17_BIT_RW 0x4000
#define M17_BIT_EN 0x2000
#define M17_BIT_D4 0x1000
#define M17_BIT_D5 0x0800
#define M17_BIT_D6 0x0400
#define M17_BIT_D7 0x0200
#define M17_BIT_LB 0x0100
#define M17_BIT_LG 0x0080
#define M17_BIT_LR 0x0040
#define M17_BIT_BZ 0x0020 //Added a buzzer on this pin
#define M17_BIT_B4 0x0010
#define M17_BIT_B3 0x0008
#define M17_BIT_B2 0x0004
#define M17_BIT_B1 0x0002
#define M17_BIT_B0 0x0001

VikiLCD::VikiLCD() {
    // Default values
    this->i2c_address      = MCP23017_ADDRESS;
    this->displaycontrol   = 0x00;
    this->displayfunction  = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS; // in case they forget to call begin() at least we have somethin
    this->displaymode      = 0x00;
    this->_numlines        = 4;
    
    // I2C com
//  this->i2c = new mbed::I2C(P0_27, P0_28);
    this->i2c = new mbed::I2C(p9, p10); // P0_0, P0_1

    i2c->frequency(60000);
    
    // configure the pins to use
    this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();

    this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();

    this->button_pause_pin.from_string(THEKERNEL->config->value( panel_checksum, button_pause_pin_checksum)->by_default("nc")->as_string())->as_input();

    paused= false;
    button_pause.pin(&this->button_pause_pin)->up_attach( this, &VikiLCD::on_pause_release);    
}

VikiLCD::~VikiLCD() {
    delete this->i2c;
}
    

void VikiLCD::init(){
    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50

    char data[2];
    
    // Setup
    this->displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    this->_backlightBits = M17_BIT_LB|M17_BIT_LG|M17_BIT_LR; // all off

    wait_ms(50);

    // now set up input/output pins in MCP23017
    data[0]= MCP23017_IODIRA;
    data[1]= 0x1F; // buttons input, all others output
    i2c->write(this->i2c_address, data, 2);


    // set the button pullups
    data[0]= MCP23017_GPPUA;
    data[1]= 0x1F;
    i2c->write(this->i2c_address, data, 2);

    data[0]= MCP23017_IODIRB;
    data[1]= 0x00; // all pins output
    i2c->write(this->i2c_address, data, 2);

    // turn leds off
    setLed(0, false);
    setLed(1, false);
    setLed(2, false);
    
    //put the LCD into 4 bit mode
    // start with a non-standard command to make it realize we're speaking 4-bit here
    // per LCD datasheet, first command is a single 4-bit burst, 0011.
    //-----
    //  we cannot assume that the LCD panel is powered at the same time as
    //  the arduino, so we have to perform a software reset as per page 45
    //  of the HD44780 datasheet - (kch)
    //-----

    // bit pattern for the burstBits function is
    //
    //  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017 
    //  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0  
    //  RS RW EN D4 D5 D6 D7 B  G  R     B4 B3 B2 B1 B0 
    for (uint8_t i=0;i < 3;i++) {
        burstBits8b((M17_BIT_EN|M17_BIT_D5|M17_BIT_D4) >> 8);
        burstBits8b((M17_BIT_D5|M17_BIT_D4) >> 8);
    }
    burstBits8b((M17_BIT_EN|M17_BIT_D5) >> 8);
    burstBits8b(M17_BIT_D5 >> 8);


    wait_ms(5); // this shouldn't be necessary, but sometimes 16MHz is stupid-fast.

    command(LCD_FUNCTIONSET | displayfunction); // then send 0010NF00 (N=lines, F=font)
    wait_ms(5); // for safe keeping...
    command(LCD_FUNCTIONSET | displayfunction); // ... twice.
    wait_ms(5); // done!

    // turn on the LCD with our defaults. since these libs seem to use personal preference, I like a cursor.
    displaycontrol = (LCD_DISPLAYON|LCD_BACKLIGHT);
    display();
    // clear it off
    clear();

    displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    command(LCD_ENTRYMODESET | displaymode);
}

// we use this to burst bits to the GPIO chip whenever we need to. avoids repetitive code.
void VikiLCD::burstBits8b(uint8_t value) {
    char data[2];
    data[0] = MCP23017_GPIOB;
    data[1]= value;
    i2c->write(this->i2c_address, data, 2);
}

// value byte order is BA
void VikiLCD::burstBits16(uint16_t value) {
    char data[3];
    data[0] = MCP23017_GPIOA;
    data[1]= value&0xFF;
    data[2]= value>>8;
    i2c->write(this->i2c_address, data, 3);
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms) 
void VikiLCD::buzz(long duration, uint16_t freq) {
    char data[2];
    int currentRegister = 0;
    // read gpio register
    data[0] = MCP23017_GPIOA;
    i2c->write(this->i2c_address, data, 1);
    i2c->read(this->i2c_address, data, 1);       // Read from selected Register
    currentRegister= data[0];
    
    duration *=1000; //convert from ms to us
    long period = 1000000 / freq; // period in us
    long elapsed_time = 0;
    while (elapsed_time < duration) {
        data[0]= MCP23017_GPIOA;
        data[1]= currentRegister |= M17_BIT_BZ;
        i2c->write(this->i2c_address, data, 2);

        wait_us(period / 2);

        data[0]= MCP23017_GPIOA;
        data[1]= currentRegister &= ~M17_BIT_BZ;
        i2c->write(this->i2c_address, data, 2);

        wait_us(period / 2);
        elapsed_time += (period);
    }
}

uint8_t VikiLCD::readButtons(void) {
    char data[2];
    data[0] = MCP23017_GPIOA;
    i2c->write(this->i2c_address, data, 1);
    i2c->read(this->i2c_address, data, 1);       // Read from selected Register

    // check the button pause
    button_pause.check_signal();
    
    return (~data[0]) & ALL_BUTTON_BITS;
}

int VikiLCD::readEncoderDelta() {
    static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t old_AB = 0;
    old_AB <<= 2;                   //remember previous state
    old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state 
    return  enc_states[(old_AB&0x0f)];
}

void VikiLCD::clear()
{
    command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
#ifndef USE_FASTMODE
    wait_us(2000);  // this command takes a long time!
#endif
}

void VikiLCD::home()
{
    command(LCD_RETURNHOME);  // set cursor position to zero
#ifndef USE_FASTMODE
    wait_us(2000);  // this command takes a long time!
#endif
}

void VikiLCD::setCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if ( row > _numlines ) row = _numlines - 1;    // we count rows starting w/0
    command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void VikiLCD::noDisplay() {
    displaycontrol &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}
void VikiLCD::display() {
    displaycontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turns the underline cursor on/off
void VikiLCD::noCursor() {
    displaycontrol &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}
void VikiLCD::cursor() {
    displaycontrol |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turn on and off the blinking cursor
void VikiLCD::noBlink() {
    displaycontrol &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}
void VikiLCD::blink() {
    displaycontrol |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// These commands scroll the display without changing the RAM
void VikiLCD::scrollDisplayLeft(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void VikiLCD::scrollDisplayRight(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void VikiLCD::leftToRight(void) {
    displaymode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This is for text that flows Right to Left
void VikiLCD::rightToLeft(void) {
    displaymode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This will 'right justify' text from the cursor
void VikiLCD::autoscroll(void) {
    displaymode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This will 'left justify' text from the cursor
void VikiLCD::noAutoscroll(void) {
    displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | displaymode);
}

void VikiLCD::command(uint8_t value) {
    send(value, 0);
}

void VikiLCD::write(char value) {
    send(value, 1);
}

// Sets the indicator leds
void VikiLCD::setLed(int led, bool onoff) {
    // LED turns on when bit is cleared
    if(onoff) {
        switch(led) {
            case LED_FAN_ON: _backlightBits &= ~M17_BIT_LR; break; // on
            case LED_HOTEND_ON: _backlightBits &= ~M17_BIT_LG; break; // on
            case LED_BED_ON: _backlightBits &= ~M17_BIT_LB; break; // on
        }
    }else{
        switch(led) {
            case LED_FAN_ON: _backlightBits |= M17_BIT_LR; break; // off
            case LED_HOTEND_ON: _backlightBits |= M17_BIT_LG; break; // off
            case LED_BED_ON: _backlightBits |= M17_BIT_LB; break; // off
        }
    }       
    burstBits16(_backlightBits);
}

// write either command or data, burst it to the expander over I2C.
void VikiLCD::send(uint8_t value, uint8_t mode) {
#ifdef USE_FASTMODE
    // polls for ready. not sure on I2C this is any faster
    
    // set Data pins as input
    char data[2];
    data[0]= MCP23017_IODIRB;
    data[1]= 0x1E;
    i2c->write(this->i2c_address, data, 2);
    uint8_t b= _backlightBits >> 8;
    burstBits8b((M17_BIT_RW>>8)|b); // RW hi,RS lo 
    char busy;
    data[0] = MCP23017_GPIOB;
    do {
        burstBits8b(((M17_BIT_RW|M17_BIT_EN)>>8)|b); // EN hi
        i2c->write(this->i2c_address, data, 1);
        i2c->read(this->i2c_address, &busy, 1); // Read D7
        burstBits8b((M17_BIT_RW>>8)|b); // EN lo
        burstBits8b(((M17_BIT_RW|M17_BIT_EN)>>8)|b); // EN hi
        burstBits8b((M17_BIT_RW>>8)|b); // EN lo
    } while ((busy&(M17_BIT_D7>>8)) != 0);

    // reset data bits as output
    data[0]= MCP23017_IODIRB;
    data[1]= 0x00;
    i2c->write(this->i2c_address, data, 2);
    burstBits8b(b); // RW lo 

#else
//  wait_us(320);
#endif
    
    // BURST SPEED, OH MY GOD
    // the (now High Speed!) I/O expander pinout
    //  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017 
    //  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0  
    //  RS RW EN D4 D5 D6 D7 B  G  R     B4 B3 B2 B1 B0 

    // n.b. RW bit stays LOW to write
    uint8_t buf = _backlightBits >> 8;
    // send high 4 bits
    if (value & 0x10) buf |= M17_BIT_D4 >> 8;
    if (value & 0x20) buf |= M17_BIT_D5 >> 8;
    if (value & 0x40) buf |= M17_BIT_D6 >> 8;
    if (value & 0x80) buf |= M17_BIT_D7 >> 8;
    
    if (mode) buf |= (M17_BIT_RS|M17_BIT_EN) >> 8; // RS+EN
    else buf |= M17_BIT_EN >> 8; // EN

    burstBits8b(buf);

    // resend w/ EN turned off
    buf &= ~(M17_BIT_EN >> 8);
    burstBits8b(buf);

    // send low 4 bits
    buf = _backlightBits >> 8;
    // send high 4 bits
    if (value & 0x01) buf |= M17_BIT_D4 >> 8;
    if (value & 0x02) buf |= M17_BIT_D5 >> 8;
    if (value & 0x04) buf |= M17_BIT_D6 >> 8;
    if (value & 0x08) buf |= M17_BIT_D7 >> 8;
    
    if (mode) buf |= (M17_BIT_RS|M17_BIT_EN) >> 8; // RS+EN
    else buf |= M17_BIT_EN >> 8; // EN

    burstBits8b(buf);

    // resend w/ EN turned off
    buf &= ~(M17_BIT_EN >> 8);
    burstBits8b(buf);
}

// We pause the system
uint32_t VikiLCD::on_pause_release(uint32_t dummy){
    if(!paused) {
        THEKERNEL->pauser->take();
        paused= true;
    }else{
        THEKERNEL->pauser->release();
        paused= false;
    }
    return 0;
}
