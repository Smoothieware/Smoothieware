/*
 * R1000A_I2C.cpp
 *
 *  Created on: Sep 17, 2016
 *      Author: sherifeid
 */

#include "R1000A_I2C.h"

R1000A_I2C::R1000A_I2C(){
    // Default Constructor
    this->i2c = new mbed::I2C(P0_27, P0_28);    // define master i2c comm class
    this->i2c->frequency(100000);               // set I2C bus freq in Hz
}

R1000A_I2C::~R1000A_I2C(){
    // Destructor
}


int R1000A_I2C::I2C_ReadREG(char I2CAddr, char REGAddr, char * data, int length){
    // perform burst register read
    int i;      // for loop variable
    // set the register to access
    this->i2c->start();
    if (this->i2c->write(I2CAddr) != 1){        // check for slave ack
        // slave I2C is not acknowledging, exit function
        this->i2c->stop();
        return -1;
    }
    this->i2c->write(REGAddr);                  // register address
    this->i2c->stop();

    // read part
    this->i2c->start();
    this->i2c->write(I2CAddr | 0x01);           // slave I2C address, with read command
    for (i=0; i<length; i++){                   // loop over every byte
        data[i] = this->i2c->read(1);
    }
    this->i2c->read(0);                         // extra dummy read for mbed I2C to stop properly
    this->i2c->stop();
    return 0;
}

int R1000A_I2C::I2C_WriteREG(char I2CAddr, char REGAddr, char * data, int length)
{
    // perform burst register read
    int i;      // for loop variable

    // set the register to access
    this->i2c->start();
    if (this->i2c->write(I2CAddr) != 0){        // check for slave ack
        // slave I2C is not acknowledging, exit function
        this->i2c->stop();
        return -1;
    }
    this->i2c->write(REGAddr);                  // register address
    for (i=0; i<length; i++){
        this->i2c->write(data[i]);              // write data one by one
    }
    this->i2c->stop();
    return 0;
}


