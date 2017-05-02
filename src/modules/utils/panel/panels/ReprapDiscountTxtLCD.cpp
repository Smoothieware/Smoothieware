/*
 * ReprapDiscountTxtLCD.cpp
 *
 *  Author: Tarmo Kople
 */

#include "ReprapDiscountTxtLCD.h"

#include "Kernel.h"
#include "checksumm.h"
#include "libs/Config.h"
#include "ConfigValue.h"

#define LCD_CMD 0
#define LCD_DATA 1

// config settings
#define panel_checksum              CHECKSUM("panel")
#define encoder_a_pin_checksum      CHECKSUM("encoder_a_pin")
#define encoder_b_pin_checksum      CHECKSUM("encoder_b_pin")
#define click_button_pin_checksum   CHECKSUM("click_button_pin")
#define pause_button_pin_checksum   CHECKSUM("pause_button_pin")
#define back_button_pin_checksum    CHECKSUM("back_button_pin")
#define buzz_pin_checksum           CHECKSUM("buzz_pin")
#define lcd4_pin_checksum           CHECKSUM("lcd4_pin")
#define lcd5_pin_checksum           CHECKSUM("lcd5_pin")
#define lcd6_pin_checksum           CHECKSUM("lcd6_pin")
#define lcd7_pin_checksum           CHECKSUM("lcd7_pin")
#define lcde_pin_checksum           CHECKSUM("lcde_pin")
#define lcdrs_pin_checksum          CHECKSUM("lcdrs_pin")

ReprapDiscountTxtLCD::ReprapDiscountTxtLCD()
{
    // configure the pins to use
    this->encoder_a_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_a_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->encoder_b_pin.from_string(THEKERNEL->config->value( panel_checksum, encoder_b_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->click_pin.from_string(THEKERNEL->config->value( panel_checksum, click_button_pin_checksum )->by_default("nc")->as_string())->as_input();
    this->pause_pin.from_string(THEKERNEL->config->value( panel_checksum, pause_button_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->back_pin.from_string(THEKERNEL->config->value( panel_checksum, back_button_pin_checksum)->by_default("nc")->as_string())->as_input();
    this->buzz_pin.from_string(THEKERNEL->config->value( panel_checksum, buzz_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcd_data_pins[0].from_string(THEKERNEL->config->value( panel_checksum, lcd4_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcd_data_pins[1].from_string(THEKERNEL->config->value( panel_checksum, lcd5_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcd_data_pins[2].from_string(THEKERNEL->config->value( panel_checksum, lcd6_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcd_data_pins[3].from_string(THEKERNEL->config->value( panel_checksum, lcd7_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcde_pin.from_string(THEKERNEL->config->value( panel_checksum, lcde_pin_checksum)->by_default("nc")->as_string())->as_output();
    this->lcdrs_pin.from_string(THEKERNEL->config->value( panel_checksum, lcdrs_pin_checksum)->by_default("nc")->as_string())->as_output();

}

ReprapDiscountTxtLCD::~ReprapDiscountTxtLCD()
{
}

void ReprapDiscountTxtLCD::write(const char* line, int len)
{
    static uint8_t row_addr[] =
    { 0x80, 0xc0, 0x94, 0xd4 };	// row start address
    char addr = row_addr[this->row] + this->col;
    writeByte(&addr, LCD_CMD);	// set cursor
    for (uint8_t n = 0; n < len; ++n)
    {
        writeByte(line + n, LCD_DATA);
    }
    this->row += len;
}

void ReprapDiscountTxtLCD::home()
{
    this->col = 0;
    this->row = 0;
}

void ReprapDiscountTxtLCD::clear()
{
    char cmd = 0x01;			// 0x01 is cls command
    writeByte(&cmd, LCD_CMD);	// cls, and set cursor to 0
    wait_us(1640);    		 	// This command takes 1.64 ms
    setCursor(0, 0);
}

void ReprapDiscountTxtLCD::display()
{
    // it is always on
}

void ReprapDiscountTxtLCD::setCursor(uint8_t col, uint8_t row)
{
    this->col = col;
    this->row = row;
}

void ReprapDiscountTxtLCD::init()
{

    char cmd;
    lcde_pin.set(1);
//	lcdrs_pin.set(0);
    wait_ms(15);
    cmd = 0x03;
    for (uint8_t i = 0; i < 3; ++i)
    {
        writeByte(&cmd, LCD_CMD);
        wait_us(1640);
    }
    cmd = 0x02;
    writeByte(&cmd, LCD_CMD);
    wait_us(40);
    cmd = 0x28;
    writeByte(&cmd, LCD_CMD); // Function set 001 BW N F - -
    cmd = 0x0C;
    writeByte(&cmd, LCD_CMD);
    cmd = 0x06;
    writeByte(&cmd, LCD_CMD); // Cursor Direction and Display Shift : 0000 01 CD S (CD 0-left, 1-right S(hift) 0-no, 1-yes
    clear();
}

void ReprapDiscountTxtLCD::writeByte(const char *b, uint8_t type)
{
    lcdrs_pin.set(type);	// set rs pin 0 cmd, 1 data
    wait_us(40);
    // set bytes 4 higher bits
    for (uint8_t i = 0; i < 4; ++i)
    {
        lcd_data_pins[i].set((*b >> (i + 4)) & 0x1);
    }
    // send
    wait_us(40);
    lcde_pin.set(0);
    wait_us(40);
    lcde_pin.set(1);
    // set bytes 4 lower bits
    for (uint8_t i = 0; i < 4; ++i)
    {
        lcd_data_pins[i].set((*b >> i) & 0x1);
    }
    // send
    wait_us(40);
    lcde_pin.set(0);
    wait_us(40);
    lcde_pin.set(1);
}

// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void ReprapDiscountTxtLCD::buzz(long duration, uint16_t freq)
{
    if (!this->buzz_pin.connected())
        return;

    duration *= 1000; //convert from ms to us
    long period = 1000000 / freq; // period in us
    long elapsed_time = 0;
    while (elapsed_time < duration)
    {
        this->buzz_pin.set(1);
        wait_us(period / 2);
        this->buzz_pin.set(0);
        wait_us(period / 2);
        elapsed_time += (period);
    }
}

int ReprapDiscountTxtLCD::readEncoderDelta()
{
    static const int8_t enc_states[] =
    { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    static uint8_t old_AB = 0;
    old_AB <<= 2;                   //remember previous state
    old_AB |= (this->encoder_a_pin.get() + (this->encoder_b_pin.get() * 2)); //add current state
    return enc_states[(old_AB & 0x0f)];
}

uint8_t ReprapDiscountTxtLCD::readButtons()
{
    uint8_t state = 0;
    state |= (this->click_pin.get() ? BUTTON_SELECT : 0);
    // check the pause button
    if (this->pause_pin.connected() && this->pause_pin.get())
        state |= BUTTON_PAUSE;
    if (this->back_pin.connected() && this->back_pin.get())
        state |= BUTTON_LEFT;
    return state;
}
