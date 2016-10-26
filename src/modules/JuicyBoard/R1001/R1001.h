/*
 * R1001.h
 *
 *  Created on: Oct 22, 2016
 *      Author: sherifeid
 */

#ifndef R1001_H
#define R1001_H

#include "modules/JuicyBoard/R1000A_I2C/R1000A_I2C.h"

#include "Pin.h"            // I/O
#include "I2C.h"            // mbed.h lib
#include "Module.h"         // for registering events
#include <string>
using std::string;

#define REG_STP            0x20        // I2C register address of temperature readback
#define REG_IDRVL          0x21        // I2C register address of drive current (lower byte)
#define REG_IDRVH          0x22        // I2C register address of drive current (upper byte)
#define REG_MCTL           0x23        // I2C register address of motor control
#define REG_MSTAT          0x24        // I2C register address of motor status
#define DRESET_DELAY       10          // driver reset delay in ms

class StreamOutput;

// R1000A class declaration goes here
class R1001 : public Module {
    public:
        // Default Constructor
        R1001();

        // module loading functions
        void on_module_loaded();                // Smoothie main module loading function
        void on_console_line_received(void *);  // console line received

        // stepper motor set functions
        void setMotorCurrent(int, int);         // set stepper motor driving current
        void setSTP(int, int);                  // set stepper motor resolution
        void resetDriver(int);                  // resets DRV8825 chip
        void setDriverSleep(int, int);          // sets DRV8825 in or out of sleep, depening on argument
        void setDecay(int, int);                // sets decay mode

        // stepper motor accessor functions
        int getMotorCurrent(int);               // reads motor current from IDRVH and IDRVL registers
        int getSTP(int);                        // reads step resolution register
        int getMCTL(int);                       // reads MCTL register
        int getMSTAT(int);                      // reads MSTAT register

    private:
        // Member variables
        R1000A_I2C i2c;                 // I2C class
};

#endif /* R1001_H */
