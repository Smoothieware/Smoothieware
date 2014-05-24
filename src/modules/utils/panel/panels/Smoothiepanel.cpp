/*
This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "Smoothiepanel.h"
#include "Kernel.h"

#include "smoothiepanel/LCDBang.h"

#include "smoothiepanel/Colors.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

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


#define LCD_WRITE       0x00
#define LCD_READ        0x01
#define LCD_ACK         0x01

Smoothiepanel::Smoothiepanel() {
    // Default values
    this->backlightval     = 0x00;
    this->displaycontrol   = 0x00;
    this->displayfunction  = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS; // in case they forget to call begin() at least we have somethin
    this->displaymode      = 0x00;
    this->_numlines        = 4;

    this->wii_connected    = false;

    // I2C com
    int i2c_pins = THEKERNEL->config->value(panel_checksum, i2c_pins_checksum)->by_default(3)->as_number();
    if(i2c_pins == 0){
        this->i2c = new mbed::I2C(P0_0, P0_1);
    }else if(i2c_pins == 1){
        this->i2c = new mbed::I2C(P0_10, P0_11);
    }else if(i2c_pins == 2){
        this->i2c = new mbed::I2C(P0_19, P0_20);
    }else{ // 3, default
        this->i2c = new mbed::I2C(P0_27, P0_28);
    }
    this->wii = new Wiichuck(this->i2c);

    this->i2c_address   = (char)THEKERNEL->config->value(panel_checksum, i2c_address_checksum)->by_default(0)->as_number();
    this->i2c_address = (this->i2c_address & 0x07) << 1;
    this->i2c_frequency = THEKERNEL->config->value(panel_checksum, i2c_frequency_checksum)->by_default(20000)->as_number();
    i2c->frequency(this->i2c_frequency);
    this->lcd_contrast  = THEKERNEL->config->value(panel_checksum, lcd_contrast_checksum)->by_default(0)->as_number();
    this->backlight_red   = THEKERNEL->config->value(panel_checksum, lcd_led_checksum)->by_default(255)->as_number();
    this->backlight_red   = THEKERNEL->config->value(panel_checksum, lcd_led_red_checksum)->by_default(this->backlight_red)->as_number();
    this->backlight_green = THEKERNEL->config->value(panel_checksum, lcd_led_green_checksum)->by_default(255)->as_number();
    this->backlight_blue  = THEKERNEL->config->value(panel_checksum, lcd_led_blue_checksum)->by_default(255)->as_number();
    this->playledval      = THEKERNEL->config->value(panel_checksum, play_led_brightness_checksum)->by_default(255)->as_number();
    this->backledval      = THEKERNEL->config->value(panel_checksum, back_led_brightness_checksum)->by_default(255)->as_number();

//  this->interrupt_pin.from_string(THEKERNEL->config->value(panel_checksum, i2c_interrupt_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_hue   = THEKERNEL->config->value(panel_checksum, encoder_led_hue_checksum)->by_default(220)->as_number();
}

Smoothiepanel::~Smoothiepanel() {
    delete this->wii;
    delete this->i2c;
}

void pca9634_init(I2C i2c, int address){
    const int leds = PCA9634_ADDRESS | (address & 0x0E);
    char cmd[2];

    // initialize led controller
    cmd[0] = 0x00;
    cmd[1] = 0x00;
    i2c.write(leds, cmd, 2);
    cmd[0] = 0x01;
    cmd[1] = 0x02;
    i2c.write(leds, cmd, 2);
    cmd[0] = 0x0C;
    cmd[1] = 0xAA;
    i2c.write(leds, cmd, 2);
    cmd[0] = 0x0D;
    i2c.write(leds, cmd, 2);
}

void pca9634_setLed(I2C *i2c, int address, char led, char val){
    const int leds = PCA9634_ADDRESS | (address & 0x0E);
    char cmd[2];

    cmd[0] = led; // lcd blue
    cmd[1] = val;
    i2c->write(leds, cmd, 2);
}

void pca9505_write(I2C *i2c, int address, char reg, char val){
    const int expander = PCA9505_ADDRESS | (address & 0x0E);
    char cmd[2];

    cmd[0] = reg;
    cmd[1] = val;
    i2c->write(expander, cmd, 2);
}

char pca9505_read(I2C *i2c, int address, char reg){
    const int expander = PCA9505_ADDRESS | (address & 0x0E);
    char cmd[1];

    cmd[0] = 0x04;
    i2c->write(expander, cmd, 1, false);
    i2c->read(expander, cmd, 1);
    return cmd[0];
}

void Smoothiepanel::init(){
    // init lcd and buzzer
    lcdbang_init(*this->i2c);
//    lcdbang_print(*this->i2c, " Smoothiepanel Beta - design by Logxen -");
    lcdbang_contrast(*this->i2c, this->lcd_contrast);

    pca9634_init(*this->i2c, this->i2c_address);
    setEncoderByHue(this->encoder_hue);
    setBacklightColor(this->backlight_red, this->backlight_green, this->backlight_blue);
    setPlayLED(this->playledval);
    setBackLED(this->backledval);

    pca9505_write(this->i2c, this->i2c_address, 0x18, 0xAA); // enable leds for button/led wing on port0
    pca9505_write(this->i2c, this->i2c_address, 0x08, 0x01); // enable leds for button/led wing on port0
//    wait_us(3000);
//    this->clear();
}

void Smoothiepanel::setLed(int led, bool on){
    // LED turns on when bit is cleared
    char saved = pca9505_read(this->i2c, this->i2c_address, 0x08);
    if(on) {
        switch(led) {
            case LED_FAN_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved | 0x40); break; // on
            case LED_HOTEND_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved | 0x10); break; // on
            case LED_BED_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved | 0x04); break; // on
        }
    }else{
        switch(led) {
            case LED_FAN_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved & ~0x40); break; // off
            case LED_HOTEND_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved & ~0x10); break; // off
            case LED_BED_ON: pca9505_write(this->i2c, this->i2c_address, 0x08, saved & ~0x04); break; // off
        }
    }
}

void Smoothiepanel::setLedBrightness(int led, int val){
    switch(led){
//        case LED_FAN_ON: this->backlight_green = val; break; // on
//        case LED_HOTEND_ON: this->backlight_red = val; break; // on
//        case LED_BED_ON: this->backlight_blue = val; break; // on
    }
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void Smoothiepanel::buzz(long duration, uint16_t freq) {
    const int expander = PCA9505_ADDRESS | this->i2c_address;
    char cmd[2];
    char saved;

    // save register state
    cmd[0] = 0x04;
    this->i2c->write(expander, cmd, 1, false);
    this->i2c->read(expander, cmd, 1);
    saved = cmd[0];

    // buzz
    cmd[0] = 0x0C;
    cmd[1] = saved & 0xFE;
    this->i2c->write(expander, cmd, 2);
    wait_ms(duration); // TODO: Make this not hold up the whole system
    cmd[1] = saved;
    this->i2c->write(expander, cmd, 2);
}

uint8_t Smoothiepanel::readButtons(void) {
    const int expander = PCA9505_ADDRESS | this->i2c_address;
    uint8_t button_bits = 0x00;
    char cmd[1];

    cmd[0] = 0x03;
    this->i2c->write(expander, cmd, 1, false);
    this->i2c->read(expander, cmd, 1);
    //cmd[0] = ~cmd[0];
    if((cmd[0] & 0x10) > 0) button_bits |= BUTTON_SELECT; // encoder click
    if((cmd[0] & 0x02) > 0) button_bits |= BUTTON_LEFT; // back button
    if((cmd[0] & 0x01) > 0) button_bits |= BUTTON_PAUSE; // play button
    if((cmd[0] & 0x20) > 0){ // wii accessory connected
        if(!this->wii_connected){
            this->wii->init_device();
            if(this->wii->device_type >= 0){
                this->wii_connected = true;
                wait_ms(100);
            }
        }
        if(this->wii_connected){
            this->wii->poll_device();
            if(this->wii->data_ready){
                if(this->wii->device_type == DEVICE_NUNCHUCK){
                    if(this->wii->SY > 192) button_bits |= BUTTON_UP;
                    else if(this->wii->SY < 64) button_bits |= BUTTON_DOWN;
                    if(this->wii->SX > 192) button_bits |= BUTTON_RIGHT;
                    else if(this->wii->SX < 64) button_bits |= BUTTON_LEFT;
                    if(!this->wii->BC) button_bits |= BUTTON_SELECT;
                    if(!this->wii->BZ) button_bits |= BUTTON_LEFT;
                }else if(this->wii->device_type == DEVICE_CLASSIC){
                    if(this->wii->LY > 192) button_bits |= BUTTON_UP;
                    else if(this->wii->LY < 64) button_bits |= BUTTON_DOWN;
                    if(this->wii->LX > 192) button_bits |= BUTTON_RIGHT;
                    else if(this->wii->LX < 64) button_bits |= BUTTON_LEFT;
                    if(!this->wii->BDU) button_bits |= BUTTON_UP;
                    else if(!this->wii->BDD) button_bits |= BUTTON_DOWN;
                    if(!this->wii->BDL) button_bits |= BUTTON_LEFT;
                    else if(!this->wii->BDR) button_bits |= BUTTON_RIGHT;
                    if(!this->wii->BA) button_bits |= BUTTON_SELECT;
                    if(!this->wii->BB) button_bits |= BUTTON_LEFT;
                }
            }else this->wii_connected = false;
        }
    }else this->wii_connected = false;

    // update the encoder color
    if(this->encoder_changed){
        if(this->encoder_hue > 360) this->encoder_hue -= 360;
        else if(this->encoder_hue < 0) this->encoder_hue += 360;
        this->encoder_changed = false;

        setEncoderByHue(this->encoder_hue);
    }

	return button_bits;
}

int Smoothiepanel::readEncoderDelta() {
	static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	static uint8_t old_AB = 0;
    int8_t state;
	old_AB <<= 2;                   //remember previous state
	old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state
    state = enc_states[(old_AB&0x0f)];
    if(state != 0){
        this->encoder_hue += state;
        this->encoder_changed = true;
    }
	return state;
}

void Smoothiepanel::clear()
{
    command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
//#ifndef USE_FASTMODE
//    wait_ms(50);  // this command takes a long time!
//#endif
}

void Smoothiepanel::home()
{
    command(LCD_RETURNHOME);  // set cursor position to zero
//#ifndef USE_FASTMODE
//    wait_us(2000);  // this command takes a long time!
//#endif
}

void Smoothiepanel::setCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if ( row > _numlines ) row = _numlines - 1;    // we count rows starting w/0
    command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void Smoothiepanel::noDisplay() {
    displaycontrol &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

void Smoothiepanel::display() {
    displaycontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turns the underline cursor on/off
void Smoothiepanel::noCursor() {
    displaycontrol &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}
void Smoothiepanel::cursor() {
    displaycontrol |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// Turn on and off the blinking cursor
void Smoothiepanel::noBlink() {
    displaycontrol &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}
void Smoothiepanel::blink() {
    displaycontrol |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | displaycontrol);
}

// These commands scroll the display without changing the RAM
void Smoothiepanel::scrollDisplayLeft(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void Smoothiepanel::scrollDisplayRight(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void Smoothiepanel::leftToRight(void) {
    displaymode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This is for text that flows Right to Left
void Smoothiepanel::rightToLeft(void) {
    displaymode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This will 'right justify' text from the cursor
void Smoothiepanel::autoscroll(void) {
    displaymode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | displaymode);
}

// This will 'left justify' text from the cursor
void Smoothiepanel::noAutoscroll(void) {
    displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | displaymode);
}

void Smoothiepanel::command(uint8_t value) {
    lcdbang_write(*this->i2c, value>>4, true);
    lcdbang_write(*this->i2c, value, true);
}

void Smoothiepanel::write(const char* line, int len) {
    for (int i = 0; i < len; ++i) {
        lcdbang_write(*this->i2c, *line++);
    }
}

// Allows to set the backlight, if the LCD backpack is used
void Smoothiepanel::setBacklight(uint8_t status) {
/*	// LED turns on when bit is cleared
	_backlightBits = M17_BIT_LB|M17_BIT_LG|M17_BIT_LR; // all off
	if (status & LED_RED) _backlightBits &= ~M17_BIT_LR; // red on
	if (status & LED_GREEN) _backlightBits &= ~M17_BIT_LG; // green on
	if (status & LED_BLUE) _backlightBits &= ~M17_BIT_LB; // blue on

	burstBits16(_backlightBits);
*/
}

void Smoothiepanel::setBacklightColor(uint8_t r, uint8_t g, uint8_t b) {
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x07; // lcd blue
    cmd[1] = b;
    this->i2c->write(leds, cmd, 2);
    cmd[0] = 0x08; // lcd green
    cmd[1] = g;
    this->i2c->write(leds, cmd, 2);
    cmd[0] = 0x09; // lcd red
    cmd[1] = r;
    this->i2c->write(leds, cmd, 2);
}

void Smoothiepanel::setBacklightByHue(int h) {
    float r, g, b;
    HSVtoRGB(&r, &g, &b, h, 1.0, 1.0);
    setBacklightColor(r*0xFF, g*0xFF, b*0xFF);
}

void Smoothiepanel::setEncoderLED(uint8_t r, uint8_t g, uint8_t b) {
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x04; // encoder red
    cmd[1] = r;
    this->i2c->write(leds, cmd, 2);
    cmd[0] = 0x05; // encoder green
    cmd[1] = g;
    this->i2c->write(leds, cmd, 2);
    cmd[0] = 0x06; // encoder blue
    cmd[1] = b;
    this->i2c->write(leds, cmd, 2);
}

void Smoothiepanel::setEncoderByHue(int h) {
    float r, g, b;
    HSVtoRGB(&r, &g, &b, h, 1.0, 1.0);
    setEncoderLED(r*0xFF, g*0xFF, b*0xFF);
}

void Smoothiepanel::setPlayLED(uint8_t v) {
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x02; // play
    cmd[1] = v;
    this->i2c->write(leds, cmd, 2);
}

void Smoothiepanel::setBackLED(uint8_t v) {
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x03; // back
    cmd[1] = v;
    this->i2c->write(leds, cmd, 2);
}

/*
// write either command or data, burst it to the expander over I2C.
void Smoothiepanel::send(uint8_t value, uint8_t mode) {
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
//	wait_us(320);
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
uint32_t Smoothiepanel::on_pause_release(uint32_t dummy){
	if(!paused) {
		THEKERNEL->pauser->take();
		paused= true;
	}else{
		THEKERNEL->pauser->release();
		paused= false;
	}
	return 0;
}
*/

