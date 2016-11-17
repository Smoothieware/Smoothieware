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

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "libs/SerialMessage.h"
#include "libs/utils.h"

#include "Gcode.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"

// define configuration checksums here
// motor current checksums here
const uint16_t motorcurrent_checksum[] = {
        CHECKSUM("stepmotor_0_current"),
        CHECKSUM("stepmotor_1_current"),
        CHECKSUM("stepmotor_2_current"),
        CHECKSUM("stepmotor_3_current"),
        CHECKSUM("stepmotor_4_current"),
        CHECKSUM("stepmotor_5_current"),
        CHECKSUM("stepmotor_6_current"),
        CHECKSUM("stepmotor_7_current"),
        CHECKSUM("stepmotor_8_current"),
        CHECKSUM("stepmotor_9_current"),
        CHECKSUM("stepmotor_10_current"),
        CHECKSUM("stepmotor_11_current"),
        CHECKSUM("stepmotor_12_current"),
        CHECKSUM("stepmotor_13_current"),
        CHECKSUM("stepmotor_14_current"),
        CHECKSUM("stepmotor_15_current")
};

// stepping resolution checksums here
const uint16_t motorstepres_checksum[] = {
        CHECKSUM("stepmotor_0_stepres"),
        CHECKSUM("stepmotor_1_stepres"),
        CHECKSUM("stepmotor_2_stepres"),
        CHECKSUM("stepmotor_3_stepres"),
        CHECKSUM("stepmotor_4_stepres"),
        CHECKSUM("stepmotor_5_stepres"),
        CHECKSUM("stepmotor_6_stepres"),
        CHECKSUM("stepmotor_7_stepres"),
        CHECKSUM("stepmotor_8_stepres"),
        CHECKSUM("stepmotor_9_stepres"),
        CHECKSUM("stepmotor_10_stepres"),
        CHECKSUM("stepmotor_11_stepres"),
        CHECKSUM("stepmotor_12_stepres"),
        CHECKSUM("stepmotor_13_stepres"),
        CHECKSUM("stepmotor_14_stepres"),
        CHECKSUM("stepmotor_15_stepres")
};

// stepping resolution checksums here
const uint16_t motordecay_checksum[] = {
        CHECKSUM("stepmotor_0_decay_mode"),
        CHECKSUM("stepmotor_1_decay_mode"),
        CHECKSUM("stepmotor_2_decay_mode"),
        CHECKSUM("stepmotor_3_decay_mode"),
        CHECKSUM("stepmotor_4_decay_mode"),
        CHECKSUM("stepmotor_5_decay_mode"),
        CHECKSUM("stepmotor_6_decay_mode"),
        CHECKSUM("stepmotor_7_decay_mode"),
        CHECKSUM("stepmotor_8_decay_mode"),
        CHECKSUM("stepmotor_9_decay_mode"),
        CHECKSUM("stepmotor_10_decay_mode"),
        CHECKSUM("stepmotor_11_decay_mode"),
        CHECKSUM("stepmotor_12_decay_mode"),
        CHECKSUM("stepmotor_13_decay_mode"),
        CHECKSUM("stepmotor_14_decay_mode"),
        CHECKSUM("stepmotor_15_decay_mode")
};

R1001::R1001(){
    // Default Constructor
    //this->i2c = new R1000A_I2C;
}

void R1001::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    load_config();
}

void R1001::load_config(){
    // this loads values from config file
    // stepper motor current settings

    int i;                  // for loop variable
    int currentval;

    // update stepper motor driver currents from config file
    for (i=1;i<16;i++) {
        // scan config for every motor
        currentval = THEKERNEL->config->value(motorcurrent_checksum[i])->as_int();
        if (currentval != 0){
            // set motor current only if non zero value at config load
            setMotorCurrent(i, currentval);
        }
    }

    // update stepping resolution mode from config
    for (i=1;i<16;i++) {
        // scan config for every motor
        currentval = THEKERNEL->config->value(motorstepres_checksum[i])->as_int();
        switch (currentval) {
        case 1:
            setSTP(i,0);
            break;
        case 2:
            setSTP(i,1);
            break;
        case 4:
            setSTP(i,2);
            break;
        case 8:
            setSTP(i,3);
            break;
        case 16:
            setSTP(i,4);
            break;
        case 32:
            setSTP(i,5);
            break;
        case 64:
            setSTP(i,6);
            break;
        case 128:
            setSTP(i,7);
            break;
        case 256:
            setSTP(i,8);
            break;
        case 512:
            setSTP(i,9);
            break;
        default:
            // ignores other values not listed
            break;
        }
    }

    // update decay setting
    string curstr;

    for (i=1;i<16;i++) {
        // scan config for every motor
        curstr = THEKERNEL->config->value(motordecay_checksum[i])->as_string();
        if (curstr == "slow") {
            setDecay(i,0);
        }
        else if (curstr == "fast") {
            setDecay(i,1);
        }
        else if (curstr == "mixed") {
            setDecay(i,2);
        }
    }
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
                    THEKERNEL->streams->printf("Stepres %d readback is 2^(%d) = %d\r\n", slotnum, stepres, 1 << stepres);
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
    char i2cbuf[2];

    i2cbuf[0] = idrvl;
    i2cbuf[1] = idrvh;
    if (this->i2c.I2C_WriteREG(slotnum, REG_IDRVL, i2cbuf,2) == 0){
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

    char i2cbuf[2];

    i2cbuf[0] = (char) stepres;
    if (this->i2c.I2C_WriteREG(slotnum, REG_STP, i2cbuf,1) == 0){
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

    char i2cbuf[2];

    // first we read the MCTL register to retain other control bits
    char MCTL = getMCTL(slotnum);

    if (sleep == 1){
        MCTL |= 4;          // set sleep bit to 1
    }
    else {
        MCTL &= 0xfb;       // set sleep bit to 0
    }

    i2cbuf[0] = MCTL;
    if (this->i2c.I2C_WriteREG(slotnum, REG_MCTL, i2cbuf,1) == 0){
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

    char i2cbuf[2];

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
    if (!(this->i2c.I2C_WriteREG(slotnum, REG_MCTL, i2cbuf,1) == 0)) {
        // output an error message
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }
}

void R1001::resetDriver(int slotnum){
    // this function resets the DRV8825 chip

    char i2cbuf[2];

    // first we read the MCTL register to retain other control bits
    char MCTL = getMCTL(slotnum);


    MCTL |= 0x80;          // set reset bit to 1

    i2cbuf[0] = MCTL;
    if (this->i2c.I2C_WriteREG(slotnum, REG_MCTL, i2cbuf,1) == 0){
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

    char i2cbuf[2];

    if (this->i2c.I2C_ReadREG(slotnum, REG_STP, i2cbuf,1) == 0){
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

    char i2cbuf[2];

    if (this->i2c.I2C_ReadREG(slotnum, REG_MCTL, i2cbuf,1) == 0){
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

    char i2cbuf[2];

    if (this->i2c.I2C_ReadREG(slotnum, REG_MSTAT, i2cbuf,1) == 0){
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

    char i2cbuf[2];

    if (this->i2c.I2C_ReadREG(slotnum, REG_IDRVL, i2cbuf,2) == 0){
        // if execution is successfull return current value
        return 256*(unsigned int)i2cbuf[1] + (unsigned int)i2cbuf[0];
    }
    else
    {
        // output an error value
        return -1;
    }
}

MotorPins getMotorPins(int slot_num){
    // this returns motor pins populated in the given slot number
    MotorPins _motorpins;
    switch (slot_num){
        case (1):
            _motorpins.dir_pin = "0.24";
            _motorpins.en_pin = "0.23";
            _motorpins.step_pin = "1.31";
            break;
        case (2):
            _motorpins.dir_pin = "2.3";
            _motorpins.en_pin = "2.2";
            _motorpins.step_pin = "2.1";
            break;
        case (3):
            _motorpins.dir_pin = "1.25";
            _motorpins.en_pin = "1.24";
            _motorpins.step_pin = "1.23";
            break;
        case (4):
            _motorpins.dir_pin = "1.29";
            _motorpins.en_pin = "1.28";
            _motorpins.step_pin = "1.27";
            break;
        case (5):
            _motorpins.dir_pin = "0.0";
            _motorpins.en_pin = "0.1";
            _motorpins.step_pin = "0.11";
            break;
        case (6):
            _motorpins.dir_pin = "2.13";
            _motorpins.en_pin = "2.12";
            _motorpins.step_pin = "2.11";
            break;
        case (7):
            _motorpins.dir_pin = "0.22";
            _motorpins.en_pin = "0.21";
            _motorpins.step_pin = "0.20";
            break;
        case (8):
            _motorpins.dir_pin = "0.25";
            _motorpins.en_pin = "0.26";
            _motorpins.step_pin = "0.3";
            break;
        case (9):
            _motorpins.dir_pin = "0.18";
            _motorpins.en_pin = "0.17";
            _motorpins.step_pin = "0.16";
            break;
        case (10):
            _motorpins.dir_pin = "2.7";
            _motorpins.en_pin = "2.6";
            _motorpins.step_pin = "2.5";
            break;
        case (11):
            _motorpins.dir_pin = "1.21";
            _motorpins.en_pin = "1.20";
            _motorpins.step_pin = "1.19";
            break;
        case (12):
            _motorpins.dir_pin = "0.5";
            _motorpins.en_pin = "0.4";
            _motorpins.step_pin = "4.29";
            break;
        case (13):
            _motorpins.dir_pin = "2.8";
            _motorpins.en_pin = "3.26";
            _motorpins.step_pin = "1.0";
            break;
        case (14):
            _motorpins.dir_pin = "1.17";
            _motorpins.en_pin = "1.16";
            _motorpins.step_pin = "1.15";
            break;
        case (15):
            _motorpins.dir_pin = "1.10";
            _motorpins.en_pin = "1.9";
            _motorpins.step_pin = "1.8";
            break;
        default:
            _motorpins.step_pin = "nc";
            _motorpins.en_pin = "nc";
            _motorpins.dir_pin = "nc";
            break;
    }
    return _motorpins;
}
