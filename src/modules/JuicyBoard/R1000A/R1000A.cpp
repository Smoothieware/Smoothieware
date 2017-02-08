#include "R1000A.h"

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
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"

// Juicyboard modules
//FIXME remove this include after done
#include "modules/JuicyBoard/R1001/R1001.h"

// define configuration checksums here

#define alpha_slot_num                        CHECKSUM("alpha_slot_num")
#define beta_slot_num                         CHECKSUM("beta_slot_num")
#define gamma_slot_num                        CHECKSUM("gamma_slot_num")

R1000A::R1000A(){
    // Default Constructor
    //this->i2c = new R1000A_I2C;
    this->ModResetPin = new Pin();                      // define new
    //this->ModResetPin->from_string("1.0");
    this->ModResetPin->from_string("3.25");
    this->ModResetPin->as_open_drain();
    this->ModResetPin->set(true);                       // set to high

    alphaslot = THEKERNEL->config->value(alpha_slot_num)->by_default(0)->as_int();
    betaslot = THEKERNEL->config->value(beta_slot_num)->by_default(0)->as_int();
    gammaslot = THEKERNEL->config->value(gamma_slot_num)->by_default(0)->as_int();
}

void R1000A::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    this->ScanI2CBus();                                 // perform initial I2C bus scan
    this->ResetMods();                                  // reset all modules on I2C bus
    this->InitPowerMon();                               // initialize power monitor
}


void R1000A::on_console_line_received(void* argument){
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);
    string possible_command = new_message.message;
    string cmd = shift_parameter(possible_command);

    if (cmd == "mod"){
        // This is a successful mod command, push the next command out
        cmd = shift_parameter(possible_command);
        if (cmd == "temp"){
            // this is a get temperature command
            cmd = shift_parameter(possible_command);        // shift the next argument, which is the slot number
            getTemp(cmd);
        }
        else if (cmd == "scan"){
            // Scan I2C bus and report
            ScanI2CBus();
            ReportI2CID();
        }
        else if (cmd == "reset"){
            // reset all modules
            ResetMods();
        }
        else if (cmd == "showconfig"){
            //FIXME this command is only for test, delete it
            // this shows configuration values for alpha/beta/gamma motor slots

//            int alphaslot = THEKERNEL->config->value(alpha_slot_num)->by_default(0)->as_int();
            //int betaslot = THEKERNEL->config->value(beta_slot_num)->by_default(0)->as_int();
            //int gammaslot = THEKERNEL->config->value(gamma_slot_num)->by_default(0)->as_int();

            THEKERNEL->streams->printf("reporting config values\r\n");
            THEKERNEL->streams->printf("alpha_slot_num %d\r\n", alphaslot);
            MotorPins CurrentMotorPins = getMotorPins(alphaslot);
            THEKERNEL->streams->printf("dir_pin: %s\r\n", CurrentMotorPins.dir_pin.c_str());
            THEKERNEL->streams->printf("beta_slot_num %d\r\n", betaslot);
            THEKERNEL->streams->printf("gamma_slot_num %d\r\n", gammaslot);
        }
        else if (cmd == "getpmoncfg"){
            // reset all modules
            getPowerMonCfg();
        }
        else if (cmd == "readpmon"){
            // reset all modules
            readPowerMon();
        }
        else if (cmd == "testfunc"){
            // test voltage conversion functions
            char tmp[2];
            tmp[0] = 0xc1;
            tmp[1] = 0x80;
            THEKERNEL->streams->printf("Converting CURR 0x%x%x = %.2f mV\r\n", tmp[0], tmp[1], evalCURR(tmp));
            tmp[0] = 0x3e;
            tmp[1] = 0x80;
            THEKERNEL->streams->printf("Converting CURR 0x%x%x = %.2f mV\r\n", tmp[0], tmp[1], evalCURR(tmp));
        }
    }
}


void R1000A::ScanI2CBus(){
    // Scan addresses 0x10 through 0x1F
    // Address range is hardcoded to match R1000A platform
    // To identify which slot matches which I2C address use the following formula
    // Slot[n] has I2C address 0x10 + (n)
    // so Slot[0] has an I2C address of 0x10, Slot[1] is 0x11 ... Slot[15] is 0x1F
    
    char i2cbuf[3];     // create a 2 byte buffer for I2C
    int i;              // for loop variable
    
    for (i=1; i<=15; i++){
        // check for slave ack
        if (i2c.I2C_WriteREG(i, 0x01, i2cbuf, 1) == 0){
            // continue reading from slave
            SlotPlatID[i] = (int)i2cbuf[0];
            i2c.I2C_ReadREG(i, 0x02, i2cbuf, 2);      // get device ID
            SlotDevID[i] = (int)i2cbuf[0];
            i2c.I2C_ReadREG(i, 0x03, i2cbuf, 2);      // get firmware version
            SlotDevFW[i] = (int)i2cbuf[0];

        }
        else{
            SlotPlatID[i] = -1;
            SlotDevID[i] = -1;
            SlotDevFW[i] = -1;
        }
    } 
}

void R1000A::ReportI2CID(){
    int i;                      // for loop variable
   
    for (i=1; i<=15; i++){
        if (SlotDevID[i] == -1){
            THEKERNEL->streams->printf("Slot %d NO CARD, ID: %d\r\n", i, SlotDevID[i]);
        }
        else if (SlotDevID[i] == -2){
            THEKERNEL->streams->printf("Slot %d NOT JuicyBoard COMPATIBLE! ID: %d\r\n", i, SlotDevID[i]);
        }
        else{
            THEKERNEL->streams->printf("Slot %d MOD #0x%x, FW 0x%x, PLATID 0x%x\r\n", i, SlotDevID[i], SlotDevFW[i], SlotPlatID[i]);
        }
    }
}

int R1000A::getSlotDevID(int SlotNum) const{
    // this function returns the slot ID
    // Slot Number ranges from 1 to 16
    return SlotDevID[SlotNum];
}

void R1000A::getTemp(string slotnum){
    // this function prints out the temperature of module attached to slotnum
    long slotn = std::strtol(slotnum.c_str(), NULL, 10);

    if ((slotn >0) && (slotn < 16)){
        // execute only if a valid slot number range between 0 and 15
        char i2cbuf[2];
        if (this->i2c.I2C_ReadREG(slotn, REG_TEMP, i2cbuf, 1) == 0){
            // execute only if reading operation is successful
            THEKERNEL->streams->printf("Slot %lu Temp : %d\r\n", slotn, i2cbuf[0]);
        }
        else
        {
            // output an error message
            THEKERNEL->streams->printf("Slot %lu did not ack!\r\n", slotn);
        }
    }
    else{
        THEKERNEL->streams->printf("Invalid slot %lu\r\n", slotn);
    }
}

void R1000A::ResetMods(void){
    // This sets the reset pin to low for a few ms
    THEKERNEL->streams->printf("Resetting Mods...\r\n");
    this->ModResetPin->set(false);
    wait_ms(RESET_DELAY);                  // reset delay
    this->ModResetPin->set(true);
}

void R1000A::InitPowerMon(void){
    // This function initializes the on board power monitor with proper parameters
    // initialize register 00 to 0x7527

    char i2cbuf[2];

    // readback power monitor config buffer
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Readback INL322 Config 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // enable all 3 channels, set sampling rate to 1.1ms for bus and shunt, enable 16x averaging
    i2cbuf[0] = 0x75;
    i2cbuf[1] = 0x27;
    if (THEKERNEL->i2c->I2C_WriteREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Successfully wrote config 0x%x%x to INL322\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to write INL322 config!\r\n");
    }

    // readback power monitor config buffer
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Readback INL322 Config 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }


}

void R1000A::getPowerMonCfg(void){
    // reads power monitor configuration register
    char i2cbuf[2];

    // read back power monitor config buffer
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 Config Register: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // readback manufacturer ID
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0xfe, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 Manufacturer ID (0x5449): 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read back die ID
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0xff, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 Die ID (0x3220): 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }
}

void R1000A::readPowerMon(void){
    // reads shut and bus voltages from all channels of the power monitor
    char i2cbuf[2];

    // read Ch1 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x01, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
//        THEKERNEL->streams->printf("INL322 CH1 CURR: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH1 CURR: %.2f mA\r\n", evalCURR(i2cbuf)/RES_CH1);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read Ch1 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x02, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 CH1 VOLT: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH1 VOLT: %d mV\r\n", evalVOLT(i2cbuf));
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read Ch2 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x03, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
//        THEKERNEL->streams->printf("INL322 CH2 CURR: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH2 CURR: %.2f mA\r\n", evalCURR(i2cbuf)/RES_CH2);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read Ch2 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x04, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 CH2 VOLT: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH2 VOLT: %d mV\r\n", evalVOLT(i2cbuf));
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read Ch3 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x05, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
//        THEKERNEL->streams->printf("INL322 CH3 CURR: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH3 CURR: %.2f mA\r\n", evalCURR(i2cbuf)/RES_CH3);
    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }

    // read Ch3 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x06, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 CH3 VOLT: 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
        THEKERNEL->streams->printf("INL322 CH3 VOLT: %d mV\r\n", evalVOLT(i2cbuf));

    }
    else{
        THEKERNEL->streams->printf("Unable to read INL322 config!\r\n");
    }
}

float R1000A::evalCURR(char * i2cbuf){
    // this function converts shunt I2C buffer data to mV
    // first check the sign bit
    char sign = i2cbuf[0] & 0x80;
    if (sign == 0){
        // sign bit is 0, number is positive
        return (float)((((unsigned int)(i2cbuf[0] & 0x7f)<<5) | (unsigned int)(i2cbuf[1] >> 3))*0.04);
    }
    else {
        // sign bit is 1, number is negative
        unsigned int adcval = ((unsigned int)(i2cbuf[0] & 0x7f) << 8) | (unsigned int)i2cbuf[1];
        adcval = adcval - 1;                // subtract 1
        adcval = (~adcval) & 0x00007fff;    // complement and mask
        adcval = adcval >> 3;               // shift 3 bits to divide by 8
        return (float)(adcval * -0.04);
    }
}

int R1000A::evalVOLT(char * i2cbuf){
    // this function converts shunt I2C buffer data to mV
    // first check the sign bit
    char sign = i2cbuf[0] & 0x80;
    if (sign == 0){
        // sign bit is 0, number is positive
        return (int)((((unsigned int)(i2cbuf[0] & 0x7f)<<5) | (unsigned int)(i2cbuf[1] >> 3))*8);
    }
    else {
        // sign bit is 1, number is negative
        unsigned int adcval = ((unsigned int)(i2cbuf[0] & 0x7f) << 8) | (unsigned int)i2cbuf[1];
        adcval = adcval - 1;                // subtract 1
        adcval = (~adcval) & 0x00007fff;    // complement and mask
        adcval = adcval >> 3;               // shift 3 bits to divide by 8
        return (int)(adcval * -8);
    }
}
