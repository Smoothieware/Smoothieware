/*
 * R1000A_I2C.h
 *
 *  Created on: Sep 17, 2016
 *      Author: sherifeid
 */

#ifndef SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_
#define SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_

#define R1000_I2C_BASE      0x10        // I2C address base for R1000A module
#define R1000_I2C_BLBASE    0x20        // I2C address base for R1000A module
#define PWRMON_BASE         0x40        // I2C address of on board power monitor

#define PWRMON_SLOT         100         // slot number for power monitor

#define EEPROM_SLOT_BASE    200         // base slot number for EEPROM
#define EEPROM_NUM_SLOTS    4           // 1kbyte EEPROM can address 4 slots (200,201,202,203)

#define EEPROM_PAGE_SIZE    16          // number of bytes/page

#define EEPROM_I2C_BASE     0x50        // on board EEPROM address base

// bootloader defines
#define TGT_CMD_CHECK_BLSTAT    0x0B    // command to check bootloader status
#define TGT_RSP_BL_MODE         0x03

#include "I2C.h"            // mbed.h lib

// This is an I2C wrapper class for low level I2C commands
class R1000A_I2C {
    public:
        // Default Constructor
        R1000A_I2C();

        // Destructor
        ~R1000A_I2C();

        // low level I2C operations
        int I2C_ReadREG(int, char, char*, int);         // burst read (using slotnum)
        int I2C_WriteREG(int, char, char*, int);        // burst write (using slotnum)
        int I2C_Read(int, char*, int);                  // burst read (using slotnum) current register

//        int I2C_BLReadREG(int, char, char*, int);       // burst read (using slotnum) current register (bootloader mode only)
//        int I2C_BLWriteREG(int, char, char*, int);      // burst write (using slotnum) current register (bootloader mode only)
//        int I2C_BLRead(int, char*, int);                // burst read (using slotnum) current register (bootloader mode only)
        int I2C_CheckAddr(char);                        // checks if I2C address acknowledges
        int I2C_CheckAck(int);                          // checks if slot acknowledges
        int I2C_CheckBLMode(int);                       // checks if the slot is in bootloader mode
//        int I2C_CheckBLAck(int);                        // checks if slot (in bootloader mode) acknowledges

    private:
        // Member variables
        mbed::I2C* i2c;                                 // i2c comm class
        char getSlotI2CAdd(int);                        // returns I2C address from slot number
//        char getBLSlotI2CAdd(int);                      // returns I2C address from slot number, used for addressing modules in bootloader mode only
};

#endif /* SRC_MODULES_JUICYBOARD_R1000A_I2C_R1000A_I2C_H_ */
