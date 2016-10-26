/*
 * R1001.cpp
 *
 *  Created on: Oct 22, 2016
 *      Author: sherifeid
 */

#include "R1001.h"
#include <string>

#include "wait_api.h"

#include "StreamOutputPool.h"
#include "Module.h"

#include "CurrentControl.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "libs/SerialMessage.h"
#include "libs/utils.h"

#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"

// define configuration checksums here


R1001::R1001(){
    // Default Constructor
    //this->i2c = new R1000A_I2C;
}

void R1001::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    // FIXME add configuration read init here
}


void R1001::on_console_line_received(void* argument){
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);
    string possible_command = new_message.message;
    string cmd = shift_parameter(possible_command);

    if (cmd == "mod"){
        cmd = shift_parameter(possible_command);        // gets the next argument, which should be r1001 command
        if ((cmd == "r1001")||(cmd == "R1001")) {
            // this is an R1001 command
            cmd = shift_parameter(possible_command);        // gets the next argument, which is the slot number
            int slotnum = atoi(cmd.c_str());
            int stepres;
            int sleep;

            if ((slotnum < 16) && (slotnum > 0)) {
                // slot number has to be valid
                cmd = shift_parameter(possible_command);        // shifts to next argument, which is the R1001 specific command
                if (cmd == "setcurrent") {
                    // set motor current
                    cmd = shift_parameter(possible_command);    // get current value in mA
                    int idrv = atoi(cmd.c_str());
                    THEKERNEL->streams->printf("Setting #%d driver current to %d mA\r\n", slotnum, idrv);
                    setMotorCurrent(slotnum, idrv);
                }
                else if (cmd == "getcurrent") {
                    // reads current from IDRVH and IDRVL registers
                    THEKERNEL->streams->printf("Motor %d Current is %dmA\r\n", slotnum, getMotorCurrent(slotnum));
                }
                else if (cmd == "setres") {
                    // sets stepper motor resolution
                    stepres = atoi(shift_parameter(possible_command).c_str());
                    setSTP(slotnum, stepres);
                }
                else if (cmd == "getres") {
                    // reads resolution setting from STP register
                    stepres = getSTP(slotnum);
                    THEKERNEL->streams->printf("Stepres readback is 2^(%d) = %d\r\n", stepres, 1 << stepres);
                }
                else if (cmd == "sleep") {
                    // set sleep bit
                    sleep = atoi(shift_parameter(possible_command).c_str());
                    setDriverSleep(slotnum, sleep);
                }
                else if (cmd == "getmctl") {
                    // returns MCTL register
                    THEKERNEL->streams->printf("MCTL #%d is 0x%x\r\n", slotnum, getMCTL(slotnum));
                }
                else if (cmd == "dreset") {
                    // resets DRV8825 chip
                    resetDriver(slotnum);
                }
                else if (cmd == "decayfast") {
                    // sets decay mode to fast
                    setDecay(slotnum, 1);
                }
                else if (cmd == "decayslow") {
                    // sets decay mode to slow
                    setDecay(slotnum, 0);
                }
                else if (cmd == "decaymixed") {
                    // sets decay mode to mixed
                    setDecay(slotnum, 2);
                }
                else if (cmd == "getmstat") {
                    // returns MCTL register
                    THEKERNEL->streams->printf("MSTAT #%d is 0x%x\r\n", slotnum, getMSTAT(slotnum));
                }
                else {
                    THEKERNEL->streams->printf("unknown R1001 command!\r\n");
                }
            }
            else {
                // slot number is not valid
                THEKERNEL->streams->printf("R1001 invalid slot # : %s\r\n", cmd.c_str());
            }
        }
    }
}

void R1001::setMotorCurrent(int slotnum, int idrv){
    // this function sets the stepper motor current value
    char idrvl = idrv & 0xff;                   // masking lower byte
    char idrvh = (idrv >> 8) & 0xff;            // masking upper byte
    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    i2cbuf[0] = idrvl;
    i2cbuf[1] = idrvh;
    if (this->i2c.I2C_WriteREG(i2caddr, REG_IDRVL, i2cbuf,2) == 0){
        // execute only if reading operation is successful
        THEKERNEL->streams->printf("Written 0x%x to IDRVL\r\n", i2cbuf[0]);
        THEKERNEL->streams->printf("Written 0x%x to IDRVH\r\n", i2cbuf[1]);
    }
    else
    {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
}

void R1001::setSTP(int slotnum, int stepres){
    // this function sets the stepper motor resolution
    // stepres is power of 2 resolution, example: if stepres = 5 resolution = 2^5 = 32

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    i2cbuf[0] = (char) stepres;
    if (this->i2c.I2C_WriteREG(i2caddr, REG_STP, i2cbuf,1) == 0){
        // execute only if reading operation is successful
        THEKERNEL->streams->printf("#%d Step resolution 2^(%d) = %d\r\n", slotnum, stepres, 1 << stepres);
    }
    else
    {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
}

void R1001::setDriverSleep(int slotnum, int sleep){
    // this function sets the sleep mode base on the value of 'sleep'
    // values for 'sleep'
    // 1 : sets driver to sleep mode
    // 0 : gets driver out of sleep mode

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    // first we read the MCTL register to retain other control bits
    char MCTL = getMCTL(slotnum);

    if (sleep == 1){
        MCTL |= 4;          // set sleep bit to 1
    }
    else {
        MCTL &= 0xfb;       // set sleep bit to 0
    }

    i2cbuf[0] = MCTL;
    if (this->i2c.I2C_WriteREG(i2caddr, REG_MCTL, i2cbuf,1) == 0){
        // execute only if reading operation is successful
        THEKERNEL->streams->printf("Set #%d Sleep to %d\r\n", slotnum, sleep);
    }
    else
    {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
}

void R1001::setDecay(int slotnum, int decay){
    // this function sets the DRV8825 decay mode based on the input argument 'decay'
    // values for 'decay'
    // 0 : slow decay mode
    // 1 : fast decay mode
    // 2 : mixed decay mode

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    // first we read the MCTL register to retain other control bits
    char MCTL = getMCTL(slotnum);
    THEKERNEL->streams->printf("Setting #%d decay to ", slotnum);

    MCTL &= 0xfc;           // mask decay bits to 00

    if (decay == 0){
        // slow decay       // leave decay bits at 00
        THEKERNEL->streams->printf("slow ");
    }
    else if (decay == 1) {
        MCTL |= 0x01;       // set decay bits to 01
        THEKERNEL->streams->printf("fast ");
    }
    else {
        MCTL |= 0x02;       // set decay bits to 10
        THEKERNEL->streams->printf("mixed ");
    }

    THEKERNEL->streams->printf("mode\r\n");
    i2cbuf[0] = MCTL;
    if (!(this->i2c.I2C_WriteREG(i2caddr, REG_MCTL, i2cbuf,1) == 0)) {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
}

void R1001::resetDriver(int slotnum){
    // this function resets the DRV8825 chip

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    // first we read the MCTL register to retain other control bits
    char MCTL = getMCTL(slotnum);


    MCTL |= 0x80;          // set reset bit to 1

    i2cbuf[0] = MCTL;
    if (this->i2c.I2C_WriteREG(i2caddr, REG_MCTL, i2cbuf,1) == 0){
        // execute only if reading operation is successful
        THEKERNEL->streams->printf("Resetting #%d motor driver...\r\n", slotnum);
        wait_ms(DRESET_DELAY);              // add some delay to wait for driver to be reset
    }
    else
    {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
    MCTL = getMCTL(slotnum);
    if ((MCTL & 0x80) == 0) {
        THEKERNEL->streams->printf("Driver successfully reset.\r\n");
    }
}

int R1001::getSTP(int slotnum){
    // this function reads the stepper motor current value

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    if (this->i2c.I2C_ReadREG(i2caddr, REG_STP, i2cbuf,1) == 0){
        // if execution is successfull return current value
        return i2cbuf[0];
    }
    else
    {
        // output an error value
        return -1;
    }
}

int R1001::getMCTL(int slotnum){
    // this function returns MCTL register value

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    if (this->i2c.I2C_ReadREG(i2caddr, REG_MCTL, i2cbuf,1) == 0){
        // if execution is successfull return current value
        return i2cbuf[0];
    }
    else
    {
        // output an error value
        return -1;
    }
}

int R1001::getMSTAT(int slotnum){
    // this function returns MSTAT register value

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    if (this->i2c.I2C_ReadREG(i2caddr, REG_MSTAT, i2cbuf,1) == 0){
        // if execution is successfull return current value
        return i2cbuf[0];
    }
    else
    {
        // output an error value
        return -1;
    }
}


int R1001::getMotorCurrent(int slotnum){
    // this function reads the stepper motor current value

    char i2caddr;
    char i2cbuf[2];
    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address

    if (this->i2c.I2C_ReadREG(i2caddr, REG_IDRVL, i2cbuf,2) == 0){
        // if execution is successfull return current value
        return 256*(unsigned int)i2cbuf[1] + (unsigned int)i2cbuf[0];
    }
    else
    {
        // output an error value
        return -1;
    }
}

