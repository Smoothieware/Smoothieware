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

            if ((slotnum < 16) && (slotnum > 0)) {
                // slot number has to be valid
                cmd = shift_parameter(possible_command);        // shifts to next argument, which is the R1001 specific command
                if (cmd == "setcurrent") {
                    // set motor current
                    cmd = shift_parameter(possible_command);    // get current value in mA
                    int idrv = atoi(cmd.c_str());
                    THEKERNEL->streams->printf("Setting driver current to %d mA\r\n", idrv);
                    SetMotorCurrent(slotnum, idrv);
                }
                else if (cmd == "getcurrent") {
                    // reads current from IDRVH and IDRVL registers
                    THEKERNEL->streams->printf("r1001 getcurrent\r\n");
                }
                else if (cmd == "setres") {
                    // sets stepper motor resolution
                    THEKERNEL->streams->printf("r1001 setres\r\n");
                }
                else if (cmd == "getres") {
                    // reads resolution setting from STP register
                    THEKERNEL->streams->printf("r1001 getres\r\n");
                }
                else if (cmd == "reset") {
                    // resets DRV8825 chip
                    THEKERNEL->streams->printf("r1001 reset\r\n");
                }
                else if (cmd == "setdecayfast") {
                    // sets decay mode to fast
                    THEKERNEL->streams->printf("r1001 setdecayfast\r\n");
                }
                else if (cmd == "setdecayslow") {
                    // sets decay mode to slow
                    THEKERNEL->streams->printf("r1001 setdecayslow\r\n");
                }
                else if (cmd == "setdecaymixed") {
                    // sets decay mode to mixed
                    THEKERNEL->streams->printf("r1001 setdecaymixed\r\n");
                }
                else {
                    THEKERNEL->streams->printf("unknown r1001 command!\r\n");
                }
            }
            else {
                // slot number is not valid
                THEKERNEL->streams->printf("R1001 invalid slot # : %s\r\n", cmd.c_str());
            }
        }
    }
}

void R1001::SetMotorCurrent(int slotnum, int idrv){
    // this function sets the stepper motor current value
    char idrvl = idrv & 0xff;                   // masking lower byte
    char idrvh = (idrv >> 8) & 0xff;            // masking upper byte
    THEKERNEL->streams->printf("IDRVL = 0x%x\r\n", idrvl);
    THEKERNEL->streams->printf("IDRVH = 0x%x\r\n", idrvh);

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
    // reading back PCA0CPH0 register
    if (this->i2c.I2C_ReadREG(i2caddr, 0x25, i2cbuf,1) == 0) {
        THEKERNEL->streams->printf("PCA0CPH0 readback 0x%x\r\n", i2cbuf[0]);
    }
    else {
        THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
    }

}

//void R1001::LoopMotorCurrent(int slotnum, int idrv){
//    // this function sets the stepper motor current value
//
//    THEKERNEL->streams->printf("Looping on all motor current steps\r\n");
//
//    char i2caddr;
//    char i2cbuf[2];
//    i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address
//
//    int i;  // for loop variable
//
//    for (i=0; i<13; i++){
//
//        idrv = 0x01 << i;
//        char idrvl = idrv & 0xff;                   // masking lower byte
//        char idrvh = (idrv >> 8) & 0xff;            // masking upper byte
//        i2cbuf[0] = idrvl;
//        i2cbuf[1] = idrvh;
//        if (this->i2c.I2C_WriteREG(i2caddr, REG_IDRVL, i2cbuf,2) == 0){
//            // execute only if reading operation is successful
//            THEKERNEL->streams->printf("%x,%x,", i2cbuf[1], i2cbuf[0]);
//        }
//        else
//        {
//            // output an error message
//            THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
//        }
//        // reading back PCA0CPH0 register
//        if (this->i2c.I2C_ReadREG(i2caddr, 0x25, i2cbuf,1) == 0) {
//            THEKERNEL->streams->printf("%x\r\n", i2cbuf[0]);
//        }
//        else {
//            THEKERNEL->streams->printf("Slot %d did not ack!\r\n", slotnum);
//        }
//        wait_ms(40);
//    }
//}

