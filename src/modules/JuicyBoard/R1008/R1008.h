/*
 * R1008.h
 *
 *  Created on: May 8, 2017
 *      Author: sherifeid
 */

#ifndef SRC_MODULES_JUICYBOARD_R1008_R1008_H_
#define SRC_MODULES_JUICYBOARD_R1008_R1008_H_

#include "TempSensor.h"
#include <mbed.h>
#include "modules/JuicyBoard/R1000A_I2C/R1000A_I2C.h"

#define RTDREGBASE      0x31            // register from which to read temperature value

class R1008 : public TempSensor{
    public:
        // Default Constructor
        R1008();
        ~R1008();

        void UpdateConfig(uint16_t module_checksum, uint16_t name_checksum);            // reads config file
        float get_temperature();                                                        // returns RTD temperature as float

        // EEPROM functions


    private:
        int slotnum;                    // which slot
        int channelnum;                 // which channel
//        R1000A_I2C* i2c;                // local instantiation of I2C class

};


#endif /* SRC_MODULES_JUICYBOARD_R1008_R1008_H_ */
