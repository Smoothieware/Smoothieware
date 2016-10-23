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
#define REG_MCTL           0x22        // I2C register address of motor control
#define REG_MSTAT          0x22        // I2C register address of motor status

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
        void SetMotorCurrent(int, int);         // set stepper motor driving current
//        void LoopMotorCurrent(int, int);         // FIXME, remove this
        void ResetDriver(void);                 // resets DRV8825 chip
        void SetDriverSleep(int);               // sets DRV8825 in or out of sleep, depening on argument
        void SetDecayMix(int);                  // sets mixed decay bit to argument
        void SetDecayFast(int);                 // sets fast decay bit to argument

        // stepper motor accessor functions
        int getSTP(void);                       // reads step resolution register
        int getIDRV(void);                      // reads drive current value from registers IDRVL & IDRVH
        int getMCTL(void);                      // reads MCTL register
        int getMSTAT(void);                     // reads MSTAT register

    private:
        // Member variables
        R1000A_I2C i2c;                 // I2C class
};

#endif /* R1001_H */
