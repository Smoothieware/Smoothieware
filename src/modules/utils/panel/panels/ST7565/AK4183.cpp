/*
 * AK4183.cpp
 *
 *  Created on: 21-06-2013
 *      Author: Wulfnor
 */

#include "AK4183.h"

AK4183::AK4183() {
	this->i2c_address = TS_ADDRESS;
	this->i2c = new mbed::I2C(p9, p10); // P0_0, P0_1

	i2c->frequency(40000);

	penIRQ.from_string("3.25")->as_input();
}

AK4183::~AK4183() {
	delete this->i2c;
}

unsigned char AK4183::read_adc(unsigned char cmd){
	char data[2];
	data[0] = cmd;
	i2c->write(this->i2c_address, data, 1);
	i2c->read(this->i2c_address, data, 1);       // Read from selected Register
	return (~data[0]);
}

bool AK4183::pen_touching(){
	return !penIRQ.get();
}
int AK4183::get_x(){
	return read_adc(0xC2);
}

int AK4183::get_y(){
	return read_adc(0xD2);
}
