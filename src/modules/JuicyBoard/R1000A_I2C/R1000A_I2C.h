/*
 * R1000A_I2C.h
 *
 *  Created on: Sep 17, 2016
 *      Author: sherifeid
 */

#ifndef SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_
#define SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_

#define R1000_I2C_BASE      0x10        // I2C address base for R1000A module

#include "I2C.h"            // mbed.h lib

// This is an I2C wrapper class for low level I2C commands
class R1000A_I2C {
    public:
        // Default Constructor
        R1000A_I2C();

        // Destructor
        ~R1000A_I2C();

        // low level I2C operations
        int I2C_ReadREG(char, char, char*, int);          // burst read
        int I2C_WriteREG(char, char, char*, int);         // burst write

        // Accessor functions
        int getSlotDevID(int) const;

    private:
        // Member variables
        mbed::I2C* i2c;                 // i2c comm class
};



#endif /* SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_ */
