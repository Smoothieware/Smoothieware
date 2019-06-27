/*
 * TFTGLCDAdapter.cpp
 *
 * TFTGLCDAdapter is external adapter based on microcontroller.
 * TFTGLCDAdapter may use color TFT LCD with different chips and different resolutions.
 * Courently it built on STM32F103C8T6 "Blue Pill" board and color TFT LCD (ILI9325) with resolution 320x240.
 * TFTGLCDAdapter uses text screen buffer insted off graphical buffer for other panels.
 * TFTGLCDAdapter has own encoder and may have up to 8 buttons (include encoder button).
 *
 * For use TFTGLCDAdapter you need in config file set "panel.enable" parameter to "true",
 * change "panel.lcd" parameter to "tft_glcd_adapter" and set proper parameters for SPI bus.
 *
 *  Created on: 25-06-2019
 *      Author: Serhiy-K
 */

#include "TFTGLCDAdapter.h"

#include "Kernel.h"
#include "platform_memory.h"
#include "Config.h"
#include "checksumm.h"
#include "StreamOutputPool.h"
#include "ConfigValue.h"
#include "utils.h"

#define panel_checksum             CHECKSUM("panel")
#define spi_channel_checksum       CHECKSUM("spi_channel")
#define spi_cs_pin_checksum        CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum     CHECKSUM("spi_frequency")
#define buzz_pin_checksum          CHECKSUM("buzz_pin")
#define contrast_checksum          CHECKSUM("contrast")

enum Commands {
    GET_SPI_DATA = 0,
    READ_BUTTONS,        // read buttons
    READ_ENCODER,        // read encoder
    LCD_WRITE,           // write to LCD
    BUZZER,              // beep buzzer
    CONTRAST,            // set contrast
    // Other commands... 0xE0 thru 0xFF
    INIT_ADAPTER= 0xFE,  // Initialize
};

#define FBSIZE      (CHARS_PER_LINE * TEXT_LINES + 2)
#define LED_MASK    0x0f
#define PIC_MASK    0x3f

TFTGLCDAdapter::TFTGLCDAdapter() {
    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(panel_checksum, spi_channel_checksum)->by_default(0)->as_number();
    PinName mosi, miso, sclk;
    if      (spi_channel == 0) { mosi = P0_18; miso = P0_17; sclk = P0_15;}
    else if (spi_channel == 1) { mosi = P0_9;  miso = P0_8;  sclk = P0_7;}
    else                       { mosi = P0_18; miso = P0_17; sclk = P0_15;}

    this->spi = new mbed::SPI(mosi, miso, sclk);
    this->spi->frequency(THEKERNEL->config->value( panel_checksum, spi_frequency_checksum)->by_default(1000000)->as_number()); //1Mhz freq
    this->cs.from_string(THEKERNEL->config->value( panel_checksum, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();

    cs.set(1);

    this->buzz_pin.from_string(THEKERNEL->config->value( panel_checksum, buzz_pin_checksum)->by_default("nc")->as_string())->as_output();

    // contrast override
    contrast = THEKERNEL->config->value(panel_checksum, contrast_checksum)->by_default(180)->as_number();

    framebuffer = (uint8_t *)AHB0.alloc(FBSIZE); // grab some memory from USB_RAM
    if (framebuffer == NULL) THEKERNEL->streams->printf("Not enough memory available for frame buffer");
}

TFTGLCDAdapter::~TFTGLCDAdapter() {
    this->cs.set(1);
    delete this->spi;
    AHB0.dealloc(framebuffer);
}
//clearing screen
void TFTGLCDAdapter::clear() {
    memset(framebuffer, ' ', FBSIZE - 2);
    framebuffer[FBSIZE - 2] = framebuffer[FBSIZE - 1] = 0;
    tx = ty = picBits = gliph_update_cnt = 0;
}
//set new text cursor position
void TFTGLCDAdapter::setCursor(uint8_t col, uint8_t row) {
    tx = col;
    ty = row;
}
// set text cursor to uper left corner
void TFTGLCDAdapter::home() {
    tx = ty = 0;
}

void TFTGLCDAdapter::display() {
    //nothing
}
//Init adapter
void TFTGLCDAdapter::init() {
    this->cs.set(0);
    this->spi->write(INIT_ADAPTER);
    // give adapter time to init
    safe_delay_ms(100);
    this->cs.set(1);
}
//send text line to buffer
void TFTGLCDAdapter::write(const char *line, int len) {
    uint8_t pos = tx + ty * CHARS_PER_LINE;
    for (int i = 0; i < len; ++i) {
        framebuffer[pos++] = line[i];
    }
}
//send flags for icons and leds
void TFTGLCDAdapter::send_pic(const unsigned char *fbstart) {
    framebuffer[FBSIZE - 2] = picBits & PIC_MASK;
    framebuffer[FBSIZE - 1] = ledBits & LED_MASK;
    if (gliph_update_cnt) gliph_update_cnt--;
    else                  picBits = 0;
    //send framebuffer to adapter
    this->cs.set(0);
    this->spi->write(LCD_WRITE);
    for (int x = 0; x < FBSIZE; x++) {
        this->spi->write(*(fbstart++));
    }
    wait_us(10);
    this->cs.set(1);
}
//refreshing screen
void TFTGLCDAdapter::on_refresh(bool now) {
    int refresh_counts = 0;
    refresh_counts++;
    // 10Hz refresh rate
    if (now || refresh_counts % 2 == 0 ) send_pic(framebuffer);
}
//set flags for icons
void TFTGLCDAdapter::bltGlyph(int x, int y, int w, int h, const uint8_t *glyph, int span, int x_offset, int y_offset) {
    if (w == 80)
        picBits = 0x01;    //draw logo
    else {
        // Update Only every 20 refreshes
        gliph_update_cnt = 20;
        switch (x) {
            case 0:   picBits |= 0x02; break; //draw hotend_on1
            case 27:  picBits |= 0x04; break; //draw hotend_on2
            case 55:  picBits |= 0x08; break; //draw hotend_on3
            case 83:  picBits |= 0x10; break; //draw bed_on
            case 111: picBits |= 0x20; break; //draw fan_state
        }
    }
}
// Sets flags for leds
void TFTGLCDAdapter::setLed(int led, bool onoff) {
    if(onoff) {
        switch(led) {
            case LED_HOTEND_ON: ledBits |= 1; break; // on
            case LED_BED_ON:    ledBits |= 2; break; // on
            case LED_FAN_ON:    ledBits |= 4; break; // on
            case LED_HOT:       ledBits |= 8; break; // on
        }
    } else {
        switch(led) {
            case LED_HOTEND_ON: ledBits &= ~1; break; // off
            case LED_BED_ON:    ledBits &= ~2; break; // off
            case LED_FAN_ON:    ledBits &= ~4; break; // off
            case LED_HOT:       ledBits &= ~8; break; // off
        }
    }
}
// cycle the buzzer pin at a certain frequency (hz) for a certain duration (ms)
void TFTGLCDAdapter::buzz(long duration, uint16_t freq) {
    if (this->buzz_pin.connected()) { //buzzer on Smoothie main board
        duration *= 1000;             //convert from ms to us
        long period = 1000000 / freq; // period in us
        long elapsed_time = 0;
        while (elapsed_time < duration) {
            this->buzz_pin.set(1);
            wait_us(period / 2);
            this->buzz_pin.set(0);
            wait_us(period / 2);
            elapsed_time += (period);
        }
    } else { //buzzer on GLCD controller board
        this->cs.set(0);
        this->spi->write(BUZZER);
        safe_delay_us(10);
        this->cs.set(1);
    }
}
//reading button state
uint8_t TFTGLCDAdapter::readButtons(void) {
    this->cs.set(0);
    this->spi->write(READ_BUTTONS);
    safe_delay_us(10);
    uint8_t b = this->spi->write(GET_SPI_DATA);
    safe_delay_us(10);
    this->cs.set(1);
    return b;
}

int TFTGLCDAdapter::readEncoderDelta() {
    this->cs.set(0);
    this->spi->write(READ_ENCODER);
    safe_delay_us(10);
    int8_t e = this->spi->write(GET_SPI_DATA);
    safe_delay_us(10);
    this->cs.set(1);
    int d = (int16_t)e;
    return d;
}

void TFTGLCDAdapter::setContrast(uint8_t c) {
    contrast = c;
    this->cs.set(0);
    this->spi->write(CONTRAST);
    safe_delay_us(10);
    this->spi->write(c);
    safe_delay_us(10);
    this->cs.set(1);
}

