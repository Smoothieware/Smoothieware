/*
 * ReprapDiscountTxtLCD.h
 *
 *  Created on: 12.10.2015
 *      Author: tarmo
 */

#ifndef SRC_MODULES_UTILS_PANEL_PANELS_REPRAPDISCOUNTTXTLCD_H_
#define SRC_MODULES_UTILS_PANEL_PANELS_REPRAPDISCOUNTTXTLCD_H_

#include "mbed.h"
#include "LcdBase.h"
#include "Pin.h"


class ReprapDiscountTxtLCD : public LcdBase {
public:
	ReprapDiscountTxtLCD();
	virtual ~ReprapDiscountTxtLCD();

    int getEncoderResolution() { return 4; }
    bool hasGraphics() { return false; }
    uint16_t get_screen_lines() { return 4; }

    uint8_t readButtons();
    int readEncoderDelta();
    void write(const char* line, int len);
    void home();
    void clear();
    void display();
    void setCursor(uint8_t col, uint8_t row);
    void init();
    void buzz(long,uint16_t);

private:
    void writeByte(const char* b, uint8_t type);
    // row & col pos
    uint8_t col;
    uint8_t row;

    // pins
    Pin encoder_a_pin;
    Pin encoder_b_pin;
    Pin click_pin;
    Pin pause_pin;
    Pin back_pin;
    Pin buzz_pin;
    Pin lcd_data_pins[4];
    Pin lcde_pin;
    Pin lcdrs_pin;
};

#endif /* SRC_MODULES_UTILS_PANEL_PANELS_REPRAPDISCOUNTTXTLCD_H_ */
