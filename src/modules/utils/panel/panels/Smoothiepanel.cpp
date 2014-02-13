/*
This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include "Smoothiepanel.h"

//#include "smoothiepanelbeta/Colors.h"

// smoothiepanel commands
#define CMD_NOP                 0x0000

#define CMD_LCD_DATA_WRITE          0x1000
#define CMD_LCD_DATA_READ           0x1100
#define CMD_LCD_CMD_WRITE           0x1200
#define CMD_LCD_CMD_READ            0x1300

#define CMD_LCD_CONTRAST_WRITE      0x1800
#define CMD_LCD_CONTRAST_READ       0x1900

#define CMD_WING1_WRITE             0x4000
#define CMD_WING1_READ              0x4100
#define CMD_WING1_SET               0x4200
#define CMD_WING1_CLEAR             0x4300
#define CMD_WING1_NOT               0x4400
#define CMD_WING1_DIR               0x4500
#define CMD_WING1_MWRITE            0x4600  // not implemented
#define CMD_WING1_MASK              0x4700  // not implemented
#define CMD_WING1_0_CON             0x4800
#define CMD_WING1_1_CON             0x4900
#define CMD_WING1_2_CON             0x4A00
#define CMD_WING1_3_CON             0x4B00
#define CMD_WING1_4_CON             0x4C00
#define CMD_WING1_5_CON             0x4D00
#define CMD_WING1_6_CON             0x4E00
#define CMD_WING1_7_CON             0x4F00

#define CMD_WING2_WRITE             0x5000
#define CMD_WING2_READ              0x5100
#define CMD_WING2_SET               0x5200
#define CMD_WING2_CLEAR             0x5300
#define CMD_WING2_NOT               0x5400
#define CMD_WING2_DIR               0x5500
#define CMD_WING2_MWRITE            0x5600  // not implemented
#define CMD_WING2_MASK              0x5700  // not implemented
#define CMD_WING2_0_CON             0x5800
#define CMD_WING2_1_CON             0x5900
#define CMD_WING2_2_CON             0x5A00
#define CMD_WING2_3_CON             0x5B00
#define CMD_WING2_4_CON             0x5C00
#define CMD_WING2_5_CON             0x5D00
#define CMD_WING2_6_CON             0x5E00
#define CMD_WING2_7_CON             0x5F00

#define CMD_EXT_WRITE               0x6000
#define CMD_EXT_READ                0x6100
#define CMD_EXT_SET                 0x6200
#define CMD_EXT_CLEAR               0x6300
#define CMD_EXT_NOT                 0x6400
#define CMD_EXT_DIR                 0x6500
#define CMD_EXT_MWRITE              0x6600  // not implemented
#define CMD_EXT_MASK                0x6700  // not implemented
#define CMD_EXT_0_CON               0x6800
#define CMD_EXT_1_CON               0x6900
#define CMD_EXT_2_CON               0x6A00
#define CMD_EXT_3_CON               0x6B00
#define CMD_EXT_4_CON               0x6C00
#define CMD_EXT_5_CON               0x6D00
#define CMD_EXT_6_CON               0x6E00
#define CMD_EXT_7_CON               0x6F00

#define CMD_PWM_INIT                0x7800
#define CMD_PWM0_WRITE              0x7900  // W2.5
#define CMD_PWM1_WRITE              0x7A00  // W2.6
#define CMD_PWM2_WRITE              0x7B00  // W2.7
#define CMD_PWM3_WRITE              0x7C00  // W1.6
#define CMD_PWM4_WRITE              0x7D00  // LCD_LED_RED
#define CMD_PWM5_WRITE              0x7E00  // LCD_LED_GREEN
#define CMD_PWM6_WRITE              0x7F00  // LCD_LED_BLUE

#define CMD_NUNCHUCK_INIT           0x8000
#define CMD_NUNCHUCK_TYPE_READ      0x8100
#define CMD_NUNCHUCK_DATA01_READ    0x8200
#define CMD_NUNCHUCK_DATA23_READ    0x8300
#define CMD_NUNCHUCK_DATA45_READ    0x8400
#define CMD_NUNCHUCK_POLL           0x8500

#define NUNCHUCK_TYPE_NUNCHUCK      0x0000
#define NUNCHUCK_TYPE_CLASSIC       0x0101

#define CMD_ENCODER_INIT            0x8600
#define CMD_ENCODER_READ            0x8700

// lcd commands
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

    // SPI Comms
    int spi_pins = THEKERNEL->config->value(panel_checksum, spi_pins_checksum)->by_default(0)->as_number();
    if(spi_pins == 1){
        this->spi = new mbed::SPI(P0_9, P0_8, P0_7); // mosi miso sck
    }else{ // default: spi_pins == 0
        this->spi = new mbed::SPI(P0_18, P0_17, P0_15); // mosi miso sck
    }
    this->spi->format(16, 0);
    this->spi->frequency(THEKERNEL->config->value(panel_checksum, spi_frequency_checksum)->by_default(100000)->as_number());
    this->spi_ssel_pin.from_string(THEKERNEL->config->value(panel_checksum, spi_ssel_pin_checksum)->by_default("0.16")->as_string())->as_output();
    this->spi_ssel_pin.set(1);

    this->lcd_contrast  = THEKERNEL->config->value(panel_checksum, lcd_contrast_checksum)->by_default(0)->as_number();
    this->backlight_red   = THEKERNEL->config->value(panel_checksum, lcd_led_checksum)->by_default(255)->as_number();
    this->backlight_red   = THEKERNEL->config->value(panel_checksum, lcd_led_red_checksum)->by_default(this->backlight_red)->as_number();
    this->backlight_green = THEKERNEL->config->value(panel_checksum, lcd_led_green_checksum)->by_default(255)->as_number();
    this->backlight_blue  = THEKERNEL->config->value(panel_checksum, lcd_led_blue_checksum)->by_default(255)->as_number();
    this->playledval      = THEKERNEL->config->value(panel_checksum, play_led_brightness_checksum)->by_default(255)->as_number();
    this->backledval      = THEKERNEL->config->value(panel_checksum, back_led_brightness_checksum)->by_default(255)->as_number();

    this->wing1_type      = get_checksum(THEKERNEL->config->value(panel_checksum, wing1_checksum)->by_default("NC")->as_string());
    this->wing2_type      = get_checksum(THEKERNEL->config->value(panel_checksum, wing2_checksum)->by_default("NC")->as_string());
}

Smoothiepanel::~Smoothiepanel() {
    delete this->spi;
}

int Smoothiepanel::send(int c) {
    this->spi_ssel_pin.set(1);
    wait_us(1);
    this->spi_ssel_pin.set(0);
    int r = this->spi->write(c);
    this->spi_ssel_pin.set(1);
    return r;
}

// initialize basic settings
void Smoothiepanel::init(){
//    this->send(CMD_NUNCHUCK_INIT + 0); // init nunchuck for manual polling
    wait_us(100000);
    this->send(CMD_PWM_INIT + 0x70); // enable pwm for lcd led pins
    wait_us(1000);
    this->send(CMD_PWM4_WRITE + 255 - this->backlight_red);
    wait_us(1000);
    this->send(CMD_PWM5_WRITE + 255 - this->backlight_green);
    wait_us(1000);
    this->send(CMD_PWM6_WRITE + 255 - this->backlight_blue);
    wait_us(1000);

    // init wings
    if(this->wing1_type == button_wing_checksum) {
        this->send(CMD_WING1_DIR + 0x55);
    }else if(this->wing1_type == encoder_wing_checksum) {
        this->send(CMD_WING1_DIR + 0x8E);
        this->send(CMD_ENCODER_INIT + 0x45);
    }
    if(this->wing2_type == button_wing_checksum) {
        this->send(CMD_WING2_DIR + 0x55);
    }else if(this->wing2_type == encoder_wing_checksum) {
        this->send(CMD_WING2_DIR + 0x8E);
        this->send(CMD_ENCODER_INIT + 0xCD);
    }
//    wait_us(3000);
}

void Smoothiepanel::setLed(int led, bool on){
/*
    // LED turns on when bit is cleared
    if(this->wing1_type == button_wing_checksum) {
        on ? this->send(CMD_WING1_CLEAR + (1<<(led*2))) : this->send(CMD_WING1_SET + (1<<(led*2)));
    }else if(this->wing1_type == encoder_wing_checksum) {
    }
    if(this->wing2_type == button_wing_checksum) {
        on ? this->send(CMD_WING2_CLEAR + (1<<(led*2))) : this->send(CMD_WING2_SET + (1<<(led*2)));
    }else if(this->wing2_type == encoder_wing_checksum) {
    }
*/
}

void Smoothiepanel::setLedBrightness(int led, int val){
//    switch(led){
//        case LED_FAN_ON: this->backlight_green = val; break; // on
//        case LED_HOTEND_ON: this->backlight_red = val; break; // on
//        case LED_BED_ON: this->backlight_blue = val; break; // on
//    }
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void Smoothiepanel::buzz(long duration, uint16_t freq) {
/*
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
*/
}

uint8_t Smoothiepanel::readButtons(void) {
    uint8_t button_bits = 0x00;
    // read and process wings
/*
    if(this->wing1_type == button_wing_checksum) {
        this->send(CMD_WING1_READ);
        int t = this->send(CMD_NOP);
        if (t&0x80) button_bits |= BUTTON_UP;
        else if (t&0x08) button_bits |= BUTTON_DOWN;
        if (t&0x20) button_bits |= BUTTON_SELECT;
        if (t&0x02) button_bits |= BUTTON_PAUSE;
    }else if(this->wing1_type == encoder_wing_checksum) {
        this->send(CMD_WING1_READ);
        int t = this->send(CMD_NOP);
        if (t&0x01) button_bits |= BUTTON_SELECT;
        if (t&0x40) button_bits |= BUTTON_PAUSE;
    }
    if(this->wing2_type == button_wing_checksum) {
        this->send(CMD_WING2_READ);
        int t = this->send(CMD_NOP);
        if (t&0x80) button_bits |= BUTTON_UP;
        else if (t&0x08) button_bits |= BUTTON_DOWN;
        if (t&0x20) button_bits |= BUTTON_SELECT;
        if (t&0x02) button_bits |= BUTTON_PAUSE;
    }else if(this->wing2_type == encoder_wing_checksum) {
        this->send(CMD_WING2_READ);
        int t = this->send(CMD_NOP);
        if (t&0x01) button_bits |= BUTTON_SELECT;
        if (t&0x40) button_bits |= BUTTON_PAUSE;
    }

    // read and process nunchuck data
    this->send(CMD_NUNCHUCK_POLL); // poll nunchuck
    wait_us(1000);
    this->send(CMD_NUNCHUCK_TYPE_READ); // read nunchuck device type (0x0000 for Nunchuck, 0x0101 for Classic, etc)
    int nunchuck_type = this->send(CMD_NUNCHUCK_DATA01_READ); // while collecting the device type send command to read first two data bytes 
    if(nunchuck_type > 0) {
        uint8_t data_buf[6];
        int t = this->send(CMD_NUNCHUCK_DATA23_READ); // while collecting data01 send command to read data23
        data_buf[0] = (t>>8)&0xFF;
        data_buf[1] = t&0xFF;
        t = this->send(CMD_NUNCHUCK_DATA45_READ); // while collecting data23 send command to read data45
        data_buf[2] = (t>>8)&0xFF;
        data_buf[3] = t&0xFF;
        t = this->send(CMD_NOP); // while collecting data45 send command nop
        data_buf[4] = (t>>8)&0xFF;
        data_buf[5] = t&0xFF;

        if(nunchuck_type == NUNCHUCK_TYPE_NUNCHUCK) { // connected device is a nunchuck
            char SX,SY;               // 8-bit joystick
//            short AX,AY,AZ;           // 10-bit accelerometer
            bool BC,BZ;               // buttons
            SX = data_buf[0];
            SY = data_buf[1];
//            AX = (data_buf[2] << 2) + ((data_buf[5] >> 2) & 0x03);
//            AY = (data_buf[3] << 2) + ((data_buf[5] >> 4) & 0x03);
//            AZ = (data_buf[5] << 2) + ((data_buf[5] >> 6) & 0x03);
            BC = (data_buf[5] >> 1) & 0x01;
            BZ = data_buf[5] & 0x01;

            if(SY > 192) button_bits |= BUTTON_UP;
            else if(SY < 64) button_bits |= BUTTON_DOWN;
            if(SX > 192) button_bits |= BUTTON_RIGHT;
            else if(SX < 64) button_bits |= BUTTON_LEFT;
            if(!BC) button_bits |= BUTTON_SELECT;
            if(!BZ) button_bits |= BUTTON_LEFT;
        }
        else if(nunchuck_type == NUNCHUCK_TYPE_CLASSIC) { // connected device is a classic controller
            char LX,LY; //,RX,RY,LT,RT;   // 6-bit left joystick, 5-bit right joystick and triggers
            bool BDU,BDD,BDL,BDR;     // d-pad buttons
//            bool BLT,BRT;             // digital click of triggers
//            bool BH,BP,BM;            // home, plus, minus buttons
            bool BA,BB; //,BX,BY,BZL,BZR; // buttons
            LX = data_buf[0] << 2;
            LY = data_buf[1] << 2;
//            RX = (data_buf[0] & 0xC0) + ((data_buf[1] & 0xC0) >> 2) + ((data_buf[2] & 0x80) >> 4);
//            RY = data_buf[2] << 3;
//            LT = ((data_buf[2] & 0x60) << 1) + ((data_buf[3] & 0xE0) >> 2);
//            RT = data_buf[3] << 3;
            BDU = data_buf[5] & 0x01;
            BDD = (data_buf[4] >> 6) & 0x01;
            BDL = (data_buf[5] >> 1) & 0x01;
            BDR = (data_buf[4] >> 7) & 0x01;
//            BLT = (data_buf[4] >> 5) & 0x01;
//            BRT = (data_buf[4] >> 1) & 0x01;
//            BH  = (data_buf[4] >> 3) & 0x01;
//            BP  = (data_buf[4] >> 2) & 0x01;
//            BM  = (data_buf[4] >> 4) & 0x01;
            BA  = (data_buf[5] >> 4) & 0x01;
            BB  = (data_buf[5] >> 6) & 0x01;
//            BX  = (data_buf[5] >> 3) & 0x01;
//            BY  = (data_buf[5] >> 5) & 0x01;
//            BZL = (data_buf[5] >> 7) & 0x01;
//            BZR = (data_buf[5] >> 2) & 0x01;

            if((LY > 192) | !BDU) button_bits |= BUTTON_UP;
            else if((LY < 64) | !BDD) button_bits |= BUTTON_DOWN;
            if((LX > 192) | !BDR) button_bits |= BUTTON_RIGHT;
            else if((LX < 64) | !BDL) button_bits |= BUTTON_LEFT;
            if(!BA) button_bits |= BUTTON_SELECT;
            if(!BB) button_bits |= BUTTON_LEFT;
        }
    }
*/
	return button_bits;
}

int Smoothiepanel::readEncoderDelta() {
/*
    this->send(CMD_ENCODER_READ + 0x01);
    return this->send(CMD_NOP);
*/
return 0;
/*
    int8_t state;
	static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
	static uint8_t old_AB = 0;
	old_AB <<= 2;                   //remember previous state
	old_AB |= ( this->encoder_a_pin.get() + ( this->encoder_b_pin.get() * 2 ) );  //add current state
    state = enc_states[(old_AB&0x0f)];
    if(state != 0){
        this->encoder_hue += state;
        this->encoder_changed = true;
    }
	return state;
*/
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
    this->send(CMD_LCD_CMD_READ);
    int r = this->send(CMD_LCD_CMD_READ);
    while((r&0x80) == 0x80) {
        wait_us(4);
        r = this->send(CMD_LCD_CMD_READ);
    }
    wait_us(4);
    this->send(CMD_LCD_CMD_WRITE + value);
}

void Smoothiepanel::write(const char* line, int len) {
    for (int i = 0; i < len; ++i) {
        this->send(CMD_LCD_CMD_READ);
        int r = this->send(CMD_LCD_CMD_READ);
        while((r&0x80) == 0x80) {
            wait_us(4);
            r = this->send(CMD_LCD_CMD_READ);
        }
        wait_us(4);
        this->send(CMD_LCD_DATA_WRITE + line[i]);
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
/*
    this->backlight_red = r;
    this->backlight_green = g;
    this->backlight_blue = b;
    this->send(CMD_PWM4_WRITE + this->backlight_red);
    this->send(CMD_PWM5_WRITE + this->backlight_green);
    this->send(CMD_PWM6_WRITE + this->backlight_blue);
*/
}

void Smoothiepanel::setEncoderLED(uint8_t r, uint8_t g, uint8_t b) {
/*
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
*/
}

void Smoothiepanel::setPlayLED(uint8_t v) {
/*
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x02; // play
    cmd[1] = v;
    this->i2c->write(leds, cmd, 2);
*/
}

void Smoothiepanel::setBackLED(uint8_t v) {
/*
    const int leds = PCA9634_ADDRESS | this->i2c_address;
    char cmd[2];

    cmd[0] = 0x03; // back
    cmd[1] = v;
    this->i2c->write(leds, cmd, 2);
*/
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

