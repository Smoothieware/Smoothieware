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

// Juicyware includes
#include <stdio.h>

// define configuration checksums here

//#define alpha_slot_num                        CHECKSUM("alpha_slot_num")
//#define beta_slot_num                         CHECKSUM("beta_slot_num")
//#define gamma_slot_num                        CHECKSUM("gamma_slot_num")

R1000A::R1000A(){
    // Default Constructor
    this->ModResetPin = new Pin();                      // define new
    this->ModResetPin->from_string("2.10");             // defining module reset pin, fixed to 2.10
    this->ModResetPin->as_open_drain();
    this->ModResetPin->set(true);                       // set pin to high
}

void R1000A::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    THEKERNEL->i2c->disablemodI2C();                    // disable user functional I2C commands during module initialization
    this->ResetMods();                                  // reset all modules on I2C bus
    this->ScanI2CBus();                                 // perform initial I2C bus scan
    this->ReportI2CID();                                // report modules
    this->ActivateModules();                            // activate populated modules
    wait_ms(10);                                        // wait after activating modules
    this->ScanI2CBus();                                 // re-scan to get ID of activated modules
    this->ReportI2CID();                                // report modules
    this->InitPowerMon();                               // initialize power monitor
    THEKERNEL->i2c->enablemodI2C();                     // enable module functional I2C after initialization, such as R1008 temp sensor readings
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
            // Scan I2C bus and report live modules
            ScanI2CBus();
            ReportI2CID();
        }
        else if (cmd == "reset"){
            // reset all modules
            ResetMods();
        }
        else if (cmd == "getpmoncfg"){
            // read power monitor configuration registers, ref INA chip datasheet 
            getPowerMonCfg();
        }
        else if (cmd == "readpmon"){
	        // reads power monitor voltage/current values
            readPowerMon();
        }
        else if (cmd == "eeread"){
            // This function reads data from EEPROM, byte by byte
            // ex: mod eeread 0x1 0x1a -> reads 0x1a (26) bytes from EEPROM starting at address 0x01
            // since the function uses c_str() to parse values, hex and dec values can be used for addr/length
            // returns the EEPROM values in decimal

            // evaluate the length of data to be read, integer
            int eeadr = (int)strtol(shift_parameter(possible_command).c_str(),NULL,0);
            int readlen = atoi(shift_parameter(possible_command).c_str());

            THEKERNEL->streams->printf("Reading %d bytes from addr 0x%x\r\n", readlen, eeadr);
            // do some checks on the input arguments
            if (readlen > 0) {
                // now the read length is > 0
                // perform byte by byte EEPROM read
                do {
                    if (eeadr <= EEPROM_NUM_SLOTS*256) {
                        // address is within a valid range
                        THEKERNEL->streams->printf("%c",this->readEEbyte(eeadr));
                        ++eeadr;      // increment address
                    }
                    --readlen;
                } while (readlen > 0);
            }
            THEKERNEL->streams->printf("\r\n");
        }
        else if (cmd == "eereadhex"){
            // This function reads data from EEPROM, byte by byte
            // ex: mod eereadhex 0x1 0x1a -> reads 0x1a (26) bytes from EEPROM starting at address 0x01
            // same as eeread but returns values in hex

            // evaluate the length of data to be read, integer
            int eeadr = (int)strtol(shift_parameter(possible_command).c_str(),NULL,0);
            int readlen = atoi(shift_parameter(possible_command).c_str());


            THEKERNEL->streams->printf("Reading %d bytes from addr 0x%x\r\n", readlen, eeadr);
            // do some checks on the input arguments
            if (readlen > 0) {
                // now the read length is > 0
                // perform byte by byte EEPROM read
                char bufout[1];
                do {
                    bufout[0] = this->readEEbyte(eeadr);
                    THEKERNEL->streams->printf(":%02X",(unsigned int)bufout[0]);
                    ++eeadr;      // increment address
                    --readlen;
                } while (readlen > 0);
            }
            THEKERNEL->streams->printf("\r\n");
        }
        else if (cmd == "eewrite"){
            // This function writes data to EEPROM, byte by byte
            // ex: mod eewrite 0x08 20 Hello I'm JuicyBoard
            // writes the first 20 bytes of string "Hello I'm JuicyBoard" in EEPROM starting at address 0x08
            // note the written values also include SPACES in the string

            // evaluate the length of data to be read, integer
            unsigned int eeadr = (unsigned int)strtol(shift_parameter(possible_command).c_str(),NULL,0);
            int writelen = strlen(possible_command.c_str());

            THEKERNEL->streams->printf("Writing EEPROM addr 0x%x\r\n", eeadr);
            THEKERNEL->streams->printf("Writing %d bytes:\r\n", writelen);
            THEKERNEL->streams->printf("%s\r\n", possible_command.c_str());
            // do some checks on the input arguments
            if (writelen > 0) {
                // now write length is > 0
                // perform byte by byte EEPROM write
                int i;
                for (i=0; i<writelen ;i++){
                    if (this->writeEEbyte(eeadr, possible_command.c_str()[i]) != 0){
                        THEKERNEL->streams->printf("EEPROM write did not ACK!\r\n");
                    }
                    ++eeadr;
                }
            }
            THEKERNEL->streams->printf("\r\n");
        }
        else if (cmd == "scani2c"){
            THEKERNEL->i2c->disablemodI2C();
            // This scans the whole range of I2C addresses for acknowledges
            // example: mod scani2c 
            unsigned char i;
            for ( i=1; i<0x80; i++){
                wait_ms(5);
                if (THEKERNEL->i2c->I2C_CheckAddr(i) == 0){
                    THEKERNEL->streams->printf("A:0x%02X Acked...\r\n",i);
                }
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "checki2c"){
            // This checks a specific I2C address for ack
            // example: mod checki2c 0x6
            //        : checks I2C address 0x6 (7b addr) for ack
            unsigned char i = (unsigned char)strtol(shift_parameter(possible_command).c_str(),NULL,0);
            THEKERNEL->i2c->disablemodI2C();
            if (THEKERNEL->i2c->I2C_CheckAddr(i) == 0){
                THEKERNEL->streams->printf("A:0x%02X Acked...\r\n",i);
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "readi2creg"){
            // Reads an register from i2c bus
            // example: mod readi2creg 2 0x40 3
            //        : reads 3 bytes from I2C register 0x40 of slot number 2
            THEKERNEL->i2c->disablemodI2C();
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            char regaddr = (char)strtol(shift_parameter(possible_command).c_str(),NULL,0);
            int size =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            int i;

            char i2cbuf[size];

            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,regaddr,i2cbuf,size) == 0){
                THEKERNEL->streams->printf("I2C Slot:%d REG:0x%02X returned ",slotnum,regaddr);
                for (i=0;i<size;i++){
                    THEKERNEL->streams->printf("- 0x%02X ",i2cbuf[i]);
                }
                THEKERNEL->streams->printf("\r\n");
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "wrhex2bl"){
            // write hex file to module (has to be in bootloader mode)
            // example: mod wrhex2bl 3 test.hex
            //        : writes parses and writes test.hex (stored in current directory in SD card) to module on slot number 3
            //        : note to get into current directory or specify full path to hex file
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            THEKERNEL->i2c->disablemodI2C();
            string hexfname = shift_parameter(possible_command);
            wrhex2bl(hexfname.c_str(), slotnum);
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "dumphex"){
            // analyzes and dumps hex from module in bootloader mode to console
            // example: mod dumphex 5
            //        : dumps flash memory content of module #5
            THEKERNEL->i2c->disablemodI2C();
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            dumphex(slotnum);
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "chkfwsig"){
            // checks firmware signature on mounted module
            // example: mod chkfwsig 5
            THEKERNEL->i2c->disablemodI2C();
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            int tmp = checkfwsig(slotnum);
            if (tmp == 0){
                THEKERNEL->streams->printf("APP FW Signature CORRECT\r\n");
            }
            else if (tmp == -2){
                THEKERNEL->streams->printf("APP FW Signature INVALID!\r\n");
            }
            else{
                THEKERNEL->streams->printf("I2C Error\r\n");
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "readrtd"){
            // Reads RTD temperature from R1008 module
            // example: mod readrtd 2
            //        : returns temperature readings from RTD module in slot #2
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);

            char i2cbuf[4];
            union tempdata {
                float f;
                char c[4];
            };

            union tempdata chtemp;

            THEKERNEL->i2c->disablemodI2C();

            chtemp.f = THEKERNEL->i2c->I2C_ReadREGfloat(slotnum, 0x32);
            THEKERNEL->streams->printf("CH1: %.3f (0x%02X%02X%02X%02X) /", chtemp.f, chtemp.c[0], chtemp.c[1], chtemp.c[2], chtemp.c[3]);

            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,0x33,i2cbuf,4) == 0){
                chtemp.c[3] = i2cbuf[0];
                chtemp.c[2] = i2cbuf[1];
                chtemp.c[1] = i2cbuf[2];
                chtemp.c[0] = i2cbuf[3];
                THEKERNEL->streams->printf("CH2: %.3f (0x%02X%02X%02X%02X)\r\n", chtemp.f, chtemp.c[0], chtemp.c[1], chtemp.c[2], chtemp.c[3]);
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }

            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,0x34,i2cbuf,4) == 0){
                chtemp.c[3] = i2cbuf[0];
                chtemp.c[2] = i2cbuf[1];
                chtemp.c[1] = i2cbuf[2];
                chtemp.c[0] = i2cbuf[3];
                THEKERNEL->streams->printf("CH1: %.3f (0x%02X%02X%02X%02X) /", chtemp.f, chtemp.c[0], chtemp.c[1], chtemp.c[2], chtemp.c[3]);
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }

            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,0x35,i2cbuf,4) == 0){
                chtemp.c[3] = i2cbuf[0];
                chtemp.c[2] = i2cbuf[1];
                chtemp.c[1] = i2cbuf[2];
                chtemp.c[0] = i2cbuf[3];
                THEKERNEL->streams->printf("CH2: %.3f (0x%02X%02X%02X%02X)\r\n", chtemp.f, chtemp.c[0], chtemp.c[1], chtemp.c[2], chtemp.c[3]);
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }

            // read RTD values from registers
            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,0x36,i2cbuf,2) == 0){
                THEKERNEL->streams->printf("CH1R: (0x%02X%02X) /", i2cbuf[0], i2cbuf[1]);
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }
            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,0x37,i2cbuf,2) == 0){
                THEKERNEL->streams->printf("CH2R: (0x%02X%02X)\r\n", i2cbuf[0], i2cbuf[1]);
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "activate"){
            // Activates module into functional mode
            // note that all I2C based modules always boot (or reset) into bootloader mode
            // this ensures that bootloader can be entered without user interaction
            // example: mod activate 3
            //        : activates module #3
            THEKERNEL->i2c->disablemodI2C();
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);

            char i2cbuf[1];

            if (THEKERNEL->i2c->I2C_ReadREG(slotnum,TGT_CMD_START_APPFW,i2cbuf,1) == 0){
                if (i2cbuf[0] == 0xFF){
                    THEKERNEL->streams->printf("Slot #%d Activated...", slotnum);
                }
                else{
                    THEKERNEL->streams->printf("ERROR Activating #%d!!", slotnum);
                }
                THEKERNEL->streams->printf("\r\n");
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }
            THEKERNEL->i2c->enablemodI2C();
        }
        else if (cmd == "readidriver"){
            // Reads an register from i2c bus
            // example: readi2creg 2 0x40 3
            //        : reads 3 bytes from I2C register 0x40 of slot number 2
            THEKERNEL->i2c->disablemodI2C();
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);

            THEKERNEL->streams->printf("Driver current: %.3f\r\n", THEKERNEL->i2c->I2C_ReadREGfloat(slotnum,0x25));

            THEKERNEL->i2c->enablemodI2C();
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
    
    THEKERNEL->streams->printf("Scanning I2C bus ...\r\n");
    for (i=1; i<=15; i++){

        // check for slave ack
        if (THEKERNEL->i2c->I2C_ReadREG(i, 0x01, i2cbuf, 1) == 0){
            // continue reading from slave
            SlotPlatID[i] = (int)i2cbuf[0];
            THEKERNEL->i2c->I2C_ReadREG(i, 0x02, i2cbuf, 2);      // get device ID
            SlotDevID[i] = (int)i2cbuf[0];
            THEKERNEL->i2c->I2C_ReadREG(i, 0x03, i2cbuf, 2);      // get firmware version
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
        if (THEKERNEL->i2c->I2C_ReadREG(slotn, REG_TEMP, i2cbuf, 1) == 0){
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

    THEKERNEL->streams->printf("Initializing power monitor ...\r\n");

    // readback power monitor config buffer
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Readback INL322 Config 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        this->i2creaderr();
    }

    // enable all 3 channels, set sampling rate to 1.1ms for bus and shunt, enable 16x averaging
//    i2cbuf[0] = 0x75;               // set 16x averaging
    i2cbuf[0] = 0x77;               // set 64x averaging
    i2cbuf[1] = 0x27;
    if (THEKERNEL->i2c->I2C_WriteREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Successfully wrote config 0x%x%x to INL322\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        this->i2creaderr();
    }

    // readback power monitor config buffer
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x00, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("Readback INL322 Config 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        this->i2creaderr();
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
        this->i2creaderr();
    }

    // readback manufacturer ID
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0xfe, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 Manufacturer ID (0x5449): 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        this->i2creaderr();
    }

    // read back die ID
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0xff, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        THEKERNEL->streams->printf("INL322 Die ID (0x3220): 0x%x%x\r\n", i2cbuf[0], i2cbuf[1]);
    }
    else{
        this->i2creaderr();
    }
}

void R1000A::readPowerMon(void){
    // reads shut and bus voltages from all channels of the power monitor
    char i2cbuf[2];

    float ch1i,ch1v,ch2i,ch2v,ch3i,ch3v;
    // initialize variables with improbable values to reflect any I2C communication error
    ch1i = -1e6;
    ch1v = -1e6;
    ch2i = -1e6;
    ch2v = -1e6;
    ch3i = -1e6;
    ch3v = -1e6;


    // read Ch1 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x01, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch1i = evalCURR(i2cbuf)/RES_CH1;
    }

    // read Ch1 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x02, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch1v = evalVOLT(i2cbuf);
    }


    // read Ch2 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x03, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch2i = evalCURR(i2cbuf)/RES_CH2;
    }

    // read Ch2 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x04, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch2v = evalVOLT(i2cbuf);
    }

    // read Ch3 shunt voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x05, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch3i = evalCURR(i2cbuf)/RES_CH3;
    }

    // read Ch3 bus voltage
    if (THEKERNEL->i2c->I2C_ReadREG(PWRMON_SLOT, 0x06, i2cbuf, 2) == 0){
        // successfully wrote config to INL chip
        ch3v = evalVOLT(i2cbuf);

    }
    THEKERNEL->streams->printf("POWERMON:%.2fmA,%.3fV,%.2fmA,%.3fV,%.2fmA,%.3fV\r\n", ch1i, ch1v, ch2i, ch2v, ch3i, ch3v);
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

float R1000A::evalVOLT(char * i2cbuf){
    // this function converts shunt I2C buffer data to mV
    // first check the sign bit
    char sign = i2cbuf[0] & 0x80;
    if (sign == 0){
        // sign bit is 0, number is positive
        return (float)((((unsigned int)(i2cbuf[0] & 0x7f)<<5) | (unsigned int)(i2cbuf[1] >> 3))*0.008);
    }
    else {
        // sign bit is 1, number is negative
        unsigned int adcval = ((unsigned int)(i2cbuf[0] & 0x7f) << 8) | (unsigned int)i2cbuf[1];
        adcval = adcval - 1;                // subtract 1
        adcval = (~adcval) & 0x00007fff;    // complement and mask
        adcval = adcval >> 3;               // shift 3 bits to divide by 8
        return (float)(adcval * -0.008);
    }
}

void R1000A::i2creaderr(void){
    // prints out I2C read error
    THEKERNEL->streams->printf("I2C Read Error!\r\n");
}

char R1000A::readEEbyte(unsigned int eeadr){
    // This function reads a byte from EEPROM and returns a char
    // identify slot number
    int slotnum = (int)(0x3 & (eeadr >> 8)) + EEPROM_SLOT_BASE;
    char memaddr = (0xff & eeadr);
    char i2cbuf[1];
    i2cbuf[0] = '%';
    // first write memory address to device
    if (THEKERNEL->i2c->I2C_ReadREG(slotnum, memaddr, i2cbuf, 1) != 0){
        i2creaderr();               // spit out an error message
    }
    return i2cbuf[0];
}

char R1000A::writeEEbyte(unsigned int eeadr, char data){
    // This function writes a byte to EEPROM and returns a status
    // if returned status is 0 then write was successfull
    // if returned status is -1 then write didn't complete, probably due to write protection
    // identify slot number
    int slotnum = (int)(0x3 & (eeadr >> 8)) + EEPROM_SLOT_BASE;
    char memaddr = (0xff & eeadr);
    char i2cbuf[1];
    i2cbuf[0] = data;

    wait_ms(EE_TW);             // add a delay to allow any previous write operation to complete

    // first write memory address to device
    if (THEKERNEL->i2c->I2C_WriteREG(slotnum, memaddr, i2cbuf, 1) != 0){

        return -1;
    }
    return 0;
}

unsigned char R1000A::retcharval(char u,char l){
    // u: upper nibble
    // l: lower nibble
    char tmparr[3] = {u,l,0};
    return (unsigned char)strtol(tmparr,NULL,16);
}

void R1000A::wrhex2bl(const char * filename, int slotnum){
    // this function reads, analyzes hex file then writes it to the module mounted on slotnum
    FILE * fp;
    int fchar = 0;
    int lineptr = 0;
    int linenum = 1;            // keep track of line number
    char line[128];
    line[0] = 0;
    int i;                      // for loop variable

    // check slot number is within proper range
    if ((slotnum > 0) && (slotnum < 16)){
        // check if the selected slot is in bootloader mode (acks and BLSTAT = 0x03)
        int chkack = THEKERNEL->i2c->I2C_CheckAck(slotnum);
        int chkbl = THEKERNEL->i2c->I2C_CheckBLMode(slotnum);
        //if ((THEKERNEL->i2c->I2C_CheckAck(slotnum) == 0) & (THEKERNEL->i2c->I2C_CheckBLMode(slotnum) == 0)){
        if ((chkack == 0) & (chkbl == 0)){
            fp = fopen(filename, "r");
            if (fp == NULL){
                THEKERNEL->streams->printf("Couldn't open file %s..\r\n", filename);
            }
            else{
                // erase application flash pages on the target device
                // 0x400 to 1DFF
                for (i=4;i<=0x1C;i+=2){
                    // loop through and erase every page
                    // Command Format:
                    // [0] Command
                    // [1] flash key code0
                    // [2] flash key code1
                    // [3] addr0 (LSB)
                    // [4] addr1 (MSB)
                    // [5] N/A

                    char i2cbuf[4];
                    i2cbuf[0] = 0;                  // no key
                    i2cbuf[1] = 0;                  // no key
                    i2cbuf[2] = 0x00;               // LSB address
                    i2cbuf[3] = (unsigned char)i;   // LSB address
                    if (THEKERNEL->i2c->I2C_WriteREG(slotnum, TGT_CMD_ERASE_FLASH_PAGE, i2cbuf, 4) == 0){
                        THEKERNEL->streams->printf("Erasing page 0x%02X00\r\n",i);
                    }
                    else{
                        THEKERNEL->streams->printf("I2C ERROR CANNOT ERASE PAGE 0x%02X00\r\n",i);
                    }
                    wait_ms(10);                    // give some delay for erase flash to execute
                    // check if page erase was successfull
                    if (THEKERNEL->i2c->I2C_Read(slotnum, i2cbuf, 1) == 0){
                        if (i2cbuf[0] == TGT_RSP_OK){
                            THEKERNEL->streams->printf("Successfully erased page 0x%02X00\r\n",i);
                        }
                        else{
                            THEKERNEL->streams->printf("ERASE PAGE FAILED! RESP=0x%02X00\r\n",i2cbuf[0]);
                        }
                    }
                    else{
                        THEKERNEL->streams->printf("I2C ERROR CANNOT READ BACK FLASH ERASE STATUS 0x%02X00\r\n",i);
                    }

                }

                // mounted module acked on the correct address, it is in bootloader mode
                THEKERNEL->streams->printf("Flashing file %s to slot #%d\r\n", filename, slotnum);
                while (fchar != EOF){
                    // line by line read operations
                    fchar = fgetc(fp);
                    if ((fchar != '\n') && (fchar != EOF)){
                        // add the character to the line
                        if (fchar != '\r'){
                            // '\r' is in case of windows generated text files
                            line[lineptr] = (char)fchar;
                            lineptr += 1;
                        }
                    }
                    else{
                        // line is complete, execute here
                        // perform checksum on line
                        // set byte address
                        // ignore empty lines
                        if (lineptr > 0){
                            // if it's a valid line, that starts with a colon
                            if (line[0] == ':'){
                                unsigned char ll = retcharval(line[1],line[2]);                     // record length
                                unsigned char ah = retcharval(line[3],line[4]);                     // address high byte
                                unsigned char al = retcharval(line[5],line[6]);                     // address low byte
                                unsigned char tt = retcharval(line[7],line[8]);                     // record type
                                unsigned char cc = retcharval(line[lineptr-2],line[lineptr-1]);     // record checksum
                                unsigned char record[ll];                                           // record content

                                // fill record contents
                                for (i=0;i<ll;i++){
                                    record[i] = retcharval(line[9+2*i],line[10+2*i]);
                                }

                                //  verify line checksum
                                unsigned char cceval = 0;
                                cceval += ll;
                                cceval += ah;
                                cceval += al;
                                cceval += tt;
                                for (i=0;i<ll;i++){
                                    cceval += record[i];
                                }
                                cceval = ~cceval;               // NOT
                                cceval += 1;                    // eval two'2 complement
                                if (cceval == cc){
                                    // checksum is good
                                    if (tt == 0x00){

                                        char i2cbuf[40];
                                        // Command Format:
                                        // [0] Command
                                        // [1] flash key code0
                                        // [2] flash key code1
                                        // [3] addr0 (LSB)
                                        // [4] addr1 (MSB)
                                        // [5] numbytes

                                        // Bytes to write:
                                        // [6] byte0
                                        // [7] byte1
                                        // [.] ...
                                        // [6+numbytes-1] byte(numbytes-1)

                                        i2cbuf[0] = 0;              // no key
                                        i2cbuf[1] = 0;              // no key
                                        i2cbuf[2] = al;             // address LSB
                                        i2cbuf[3] = ah;             // address MSB
                                        i2cbuf[4] = ll;             // record length

                                        for (i=0;i<ll;i++){
                                            i2cbuf[5+i] = record[i];
                                        }
                                        // now fire off i2c command
                                        if (THEKERNEL->i2c->I2C_WriteREG(slotnum, TGT_CMD_WRITE_FLASH_BYTES, i2cbuf, ll+5) == 0){
                                            THEKERNEL->streams->printf("FLASHED %d > %02X%02X%02X%02X",linenum,ll,ah,al,tt);
                                            for (i=0;i<ll;i++){
                                                THEKERNEL->streams->printf("%02X",record[i]);
                                            }
                                            THEKERNEL->streams->printf("-%02X\r\n",cc);
                                        }
                                        else{
                                            THEKERNEL->streams->printf("I2C ERROR, CANNOT FLASH line %d\r\n",linenum);
                                        }
                                        wait_ms(3);         // extra delay for writing to flash
                                    }
                                }
                                else{
                                    // checksum is not good, error
                                    THEKERNEL->streams->printf(" CHEKSUM ERROR (line %d)> %02X%02X%02X%02X",linenum,ll,ah,al,tt);
                                    for (i=0;i<ll;i++){
                                        THEKERNEL->streams->printf("%02X",record[i]);
                                    }
                                    THEKERNEL->streams->printf("-%02X!=%02X\r\n",cc,cceval);
                                    THEKERNEL->streams->printf("STOPPING FLASH !!!\r\n");
                                    fchar = -1;         // force exit from while loop
                                }
                            }
                        }
                        lineptr = 0;
                        ++linenum;
                    }
                }
            }
            fclose(fp);
        }
        else{
            // module is not mounted or isn't in bootloader mode
            THEKERNEL->streams->printf("Module in slot #%d isn't in bootloader mode or maybe not mounted\r\n",slotnum);
            THEKERNEL->streams->printf("CheckAck    : %d\r\n", chkack);
            THEKERNEL->streams->printf("CheckBLMode : %d\r\n", chkbl);
        }
    }
    else{
        THEKERNEL->streams->printf("Invalid slot #%d\r\n",slotnum);
    }
}

void R1000A::dumphex(int slotnum){
    // This function dumps the module flash memory to the console
    // the module has to be in bootloader mode

    char i2cbuf[30];        // I2C buffer


    if ((slotnum > 0) && (slotnum < 16)){
        // check if the selected slot acks and is in bootloader mode
//        if (THEKERNEL->i2c->I2C_ReadREG(slotnum, TGT_CMD_CHECK_BLSTAT, i2cbuf, 1) == 0){
//            THEKERNEL->streams->printf("BLSTAT: 0x%02X\r\n",i2cbuf[0]);
//        }
        int chkack = THEKERNEL->i2c->I2C_CheckAck(slotnum);
        int chkbl = THEKERNEL->i2c->I2C_CheckBLMode(slotnum);
        //if ((THEKERNEL->i2c->I2C_CheckAck(slotnum) == 0) & (THEKERNEL->i2c->I2C_CheckBLMode(slotnum) == 0)){
        if ((chkack == 0) & (chkbl == 0)){
            // cycle from the start address, 16 bytes at a time
            int i;
            for (i=MOD_FADDR_START;i<=MOD_FADDR_END;i+=16){
                // read 16 bytes through I2C
                // Command Format:

                // [0] Command
                // [1] flash key code0
                // [2] flash key code1
                // [3] addr0 (LSB)
                // [4] addr1 (MSB)
                // [5] numbytes

                // Response:
                // [0] Response code
                // [1] byte0
                // [2] byte1
                // [.] ...
                // [numbytes] byte(numbytes-1)

                i2cbuf[0] = 0x0;                                // 0 keys
                i2cbuf[1] = 0x0;
                i2cbuf[2] = (char)(i & 0xFF);                   // Address lower byte
                i2cbuf[3] = (char)((i >> 8) & 0xFF);            // Address upper byte
                i2cbuf[4] = 16;                                 // read 16 bytes
                // now fire off i2c command
                if (THEKERNEL->i2c->I2C_WriteREG(slotnum, TGT_CMD_READ_FLASH_BYTES, i2cbuf, 20) == 0){
                    // no I2C error, now we add some delay and read the response
                    wait_ms(10);
                    if (THEKERNEL->i2c->I2C_ReadREG(slotnum, TGT_CMD_READ_FLASH_BYTES, i2cbuf, 17) == 0){
                        if (i2cbuf[0] == TGT_RSP_OK){
                            // response is ok, now we dump values
                            int j;
                            THEKERNEL->streams->printf(":10%04X00",i);
                            //  calculate checksum
                            unsigned char cceval = 0;
                            cceval += 0x10;
                            cceval += (i >> 8) & 0xFF;
                            cceval += i & 0xFF;
                            cceval += 0x00;

                            for (j=1;j<17;j++){
                                THEKERNEL->streams->printf("%02X",i2cbuf[j]);
                                cceval += i2cbuf[j];
                            }

                            cceval = ~cceval;               // NOT
                            cceval += 1;                    // eval two'2 complement

                            THEKERNEL->streams->printf("%02X\r\n",cceval);
                        }
                    }
                    else{
                        THEKERNEL->streams->printf("I2C ERROR - FLASH READBACK\r\n");
                    }

                }
                else{
                    // I2C error
                    THEKERNEL->streams->printf("I2C ERROR - CAN'T READ FLASH\r\n");
                }

            }
        }
        else{
            // module is not mounted or isn't in bootloader mode
            THEKERNEL->streams->printf("Module in slot #%d isn't in bootloader mode or maybe not mounted\r\n",slotnum);
            THEKERNEL->streams->printf("CheckAck    : %d\r\n", chkack);
            THEKERNEL->streams->printf("CheckBLMode : %d\r\n", chkbl);
        }
    }
    else{
        THEKERNEL->streams->printf("Invalid slot #%d\r\n",slotnum);
    }
}

int R1000A::checkfwsig(int slotnum){
    // This function checks if firmware signature is correct
    // returns:
    //      0 : signature is correct
    //     -1 : I2C communication error, didn't ack
    //     -2 : signature is invalid

    char i2cbuf[1];
    if (THEKERNEL->i2c->I2C_ReadREG(slotnum, TGT_CMD_CHECK_SIG, i2cbuf, 1) == 0){
        if (i2cbuf[0] == TGT_RSP_OK){
            return 0;
        }
        else{
            return -2;
        }
    }
    else{
        return -1;
    }

}

void R1000A::ActivateModules(void){
    // This scans for modules in bootloader mode and activates them
    int i;          // for loop variable
    char i2cbuf[3];     // create a 2 byte buffer for I2C

    THEKERNEL->i2c->disablemodI2C();
    for (i=1; i<=15; i++){
        // check for slave ack
        if (SlotPlatID[i] == TGT_RSP_UNSUPPORTED_CMD){
            // module is in bootloader mode
            if (THEKERNEL->i2c->I2C_ReadREG(i,TGT_CMD_START_APPFW,i2cbuf,1) == 0){
                if (i2cbuf[0] == 0xFF){
                    THEKERNEL->streams->printf("Slot #%d Activated...", i);
                    wait_ms(2);         // delay for module to boot up
                }
                else{
                    THEKERNEL->streams->printf("ERROR Activating #%d!!", 1);
                }
                THEKERNEL->streams->printf("\r\n");
            }
            else{
                THEKERNEL->streams->printf("I2C ERROR!\r\n");
            }
        }
    }
    THEKERNEL->i2c->enablemodI2C();
}
