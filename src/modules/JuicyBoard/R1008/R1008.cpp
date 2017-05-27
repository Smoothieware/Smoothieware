/*
 * R1008.cpp
 *
 *  Created on: May 8, 2017
 *      Author: sherifeid
 */

#include "R1008.h"
#include "libs/Kernel.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

//#include "StreamOutputPool.h"           //FIXME, only for debug, remove

#define slotnum_checksum CHECKSUM("slot")
#define channelnum_checksum CHECKSUM("channel")

R1008::R1008(){
    // Default Constructor
    // do nothing
    slotnum = 0;                // init to invalid value
    channelnum = 0;             // init to invalid value
}

R1008::~R1008(){
}

void R1008::UpdateConfig(uint16_t module_checksum, uint16_t name_checksum){
    // load slot and channel numbers from config file
    slotnum = THEKERNEL->config->value(module_checksum, name_checksum, slotnum_checksum)->by_default(0)->as_int();
    channelnum = THEKERNEL->config->value(module_checksum, name_checksum, channelnum_checksum)->by_default(0)->as_int();
}

float R1008::get_temperature(){
    // reads temperature from R1008
    if (slotnum == 0){
        return -100.1;
    }
    if (channelnum == 0){
        return -101.1;
    }

    char i2cbuf[4];
    // create a datastructure to convert between char[] and float
    union tempdata {
        float f;
        char c[5];
    };

    union tempdata chtemp;

    if (THEKERNEL->i2c->ismodI2Cenabled() == 1){
        if (THEKERNEL->i2c->I2C_ReadREG(slotnum,RTDREGBASE + channelnum,i2cbuf,4) == 0){
            chtemp.c[3] = i2cbuf[0];
            chtemp.c[2] = i2cbuf[1];
            chtemp.c[1] = i2cbuf[2];
            chtemp.c[0] = i2cbuf[3];
        }
        else{
            chtemp.f = -90.1;
        }
    }
    else{
        chtemp.f = -50.1;
    }

    return chtemp.f;
}
