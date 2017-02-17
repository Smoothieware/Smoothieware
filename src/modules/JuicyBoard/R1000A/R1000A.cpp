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
//#include <string.h>
//#include <stdlib.h>
#include <stdio.h>
//#include <sstream>


// define configuration checksums here

//#define alpha_slot_num                        CHECKSUM("alpha_slot_num")
//#define beta_slot_num                         CHECKSUM("beta_slot_num")
//#define gamma_slot_num                        CHECKSUM("gamma_slot_num")

R1000A::R1000A(){
    // Default Constructor
    this->ModResetPin = new Pin();                      // define new
    //this->ModResetPin->from_string("1.0");
    this->ModResetPin->from_string("2.10");
    this->ModResetPin->as_open_drain();
    this->ModResetPin->set(true);                       // set to high
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
        else if (cmd == "getpmoncfg"){
            // reset all modules
            getPowerMonCfg();
        }
        else if (cmd == "readpmon"){
            // reset all modules
            readPowerMon();
        }
        else if (cmd == "printarg"){
            //FIXME for test, delete
            // This function prints out the string argument that follows
//            string nextarg = shift_parameter(possible_command);
            THEKERNEL->streams->printf("length:%d\r\n",possible_command.length());
            THEKERNEL->streams->printf("strlen:%d\r\n",(unsigned)strlen(possible_command.c_str()));
            THEKERNEL->streams->printf("%s\r\n",possible_command.c_str());
        }
        else if (cmd == "eeread"){
            // This function reads data from EEPROM, byte by byte

            // evaluate the length of data to be read, integer
            int readlen = atoi(shift_parameter(possible_command).c_str());
            int eeadr = (int)strtol(shift_parameter(possible_command).c_str(),NULL,0);

            THEKERNEL->streams->printf("Reading %d bytes from addr 0x%x\r\n", readlen, eeadr);
            // do some checks on the input arguments
            if (readlen > 0) {
                // now the read length is > 0
                // perform byte by byte EEPROM read
//                char bufout;
                do {
                    if (eeadr <= EEPROM_NUM_SLOTS*256) {
                        // address is within a valid range
//                        bufout[0] = this->readEEbyte(eeadr);
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

            // evaluate the length of data to be read, integer
            int readlen = atoi(shift_parameter(possible_command).c_str());
            int eeadr = (int)strtol(shift_parameter(possible_command).c_str(),NULL,0);

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
                // This function reads data from EEPROM, byte by byte

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
            // This scans the whole range of I2C addresses for acknowledges
            unsigned char i;
            for ( i=1; i<0x80; i++){
                wait_ms(5);
                if (THEKERNEL->i2c->I2C_CheckAddr(i) == 0){
                    THEKERNEL->streams->printf("A:0x%02X Acked...\r\n",i);
                }
            }
        }
        else if (cmd == "checki2c"){
            // This scans the whole range of I2C addresses for acknowledges
            unsigned char i = (unsigned char)strtol(shift_parameter(possible_command).c_str(),NULL,0);

            if (THEKERNEL->i2c->I2C_CheckAddr(i) == 0){
                THEKERNEL->streams->printf("A:0x%02X Acked...\r\n",i);
            }
        }
        else if (cmd == "dumphex"){
            // analyzes and dumps hex file to console
            //FIXME delete this command
            dumphex(shift_parameter(possible_command).c_str());
        }
        else if (cmd == "wrhex2bl"){
            // analyzes and dumps hex file to console
            //FIXME delete this command
            int slotnum =  (int)strtol(shift_parameter(possible_command).c_str(), NULL, 10);
            string hexfname = shift_parameter(possible_command);
            wrhex2bl(hexfname.c_str(), slotnum);
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

void R1000A::dumphex(const char * filename){
    // this function reads, analyzes hex file then dumps results to console
    FILE * fp;
//    char * line = NULL;
//    char * line = "Dumping hex...";
//    size_t len = 0;
//    ssize_t read;
    int fchar = 0;
    int lineptr = 0;
    char line[128];
    line[0] = 0;

    fp = fopen(filename, "r");
    if (fp == NULL){
        THEKERNEL->streams->printf("Couldn't open file %s..\r\n", filename);
    }
    else{
        THEKERNEL->streams->printf("Opening file %s..\r\n", filename);
        while (fchar != EOF){
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
                        int i;
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
                        }
                        else{
                            // checksum is not good, error
                        }

                        THEKERNEL->streams->printf("> %02X%02X%02X%02X-",ll,ah,al,tt);
                        for (i=0;i<ll;i++){
                            THEKERNEL->streams->printf("%02X",record[i]);
                        }
                        THEKERNEL->streams->printf("-%02X-%02X\r\n",cc,cceval);
                    }
                }
                lineptr = 0;
            }
        }
    }
    fclose(fp);
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

    fp = fopen(filename, "r");
    if (fp == NULL){
        THEKERNEL->streams->printf("Couldn't open file %s..\r\n", filename);
    }
    else{
        // check slot number is within proper range
        // check if the selected slot is in bootloader mode (acks on 0x2X address)
        // (optional) check to bootloader version??

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
                        int i;
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
                            THEKERNEL->streams->printf("Flashing line \r\n");
                        }
                        else{
                            // checksum is not good, error
                            THEKERNEL->streams->printf(" CHEKSUM ERROR (line %d)> %02X%02X%02X%02X",linenum,ll,ah,al,tt);
                            for (i=0;i<ll;i++){
                                THEKERNEL->streams->printf("%02X",record[i]);
                            }
                            THEKERNEL->streams->printf("%02X\r\n",cc);
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
