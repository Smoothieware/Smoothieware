/*
 * AK4183.h
 *
 *  Created on: 21-06-2013
 *      Author: Wulfnor
 */

#ifndef AK4183_H_
#define AK4183_H_

#include "libs/Kernel.h"
#include "mbed.h"
#include "libs/Pin.h"

#define TS_ADDRESS 0x90

class AK4183 {
public:
	AK4183();
	virtual ~AK4183();
	int get_x();
	int get_y();
	bool pen_touching();
private:
	Pin pen;
	mbed::I2C* i2c;
	char i2c_address;
	Pin penIRQ;
	unsigned char read_adc(unsigned char cmd);
	int read_x();
	int read_y();
};

#endif /* AK4183_H_ */
