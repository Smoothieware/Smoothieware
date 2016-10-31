#include "R1000A.h"

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



R1000A::R1000A(){
    // Default Constructor
    //this->i2c = new R1000A_I2C;
    this->ModResetPin = new Pin();                      // define new
    //this->ModResetPin->from_string("1.0");
    this->ModResetPin->from_string("3.25");
    this->ModResetPin->as_open_drain();
    this->ModResetPin->set(true);                       // set to high
}

void R1000A::on_module_loaded(){
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    this->ScanI2CBus();                                 // perform initial I2C bus scan
    this->ResetMods();                                  // reset all modules on I2C bus
    // FIXME add any config init here
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
    char i2caddr;       // current I2C address
    
    for (i=0; i<=15; i++){
        i2caddr = (R1000_I2C_BASE + i) << 1;      // shift 1 to left to get 8-bit address
        // check for slave ack
        if (i2c.I2C_ReadREG(i2caddr, 0x01, i2cbuf, 1) == 0){
            // continue reading from slave
            SlotPlatID[i] = (int)i2cbuf[0];
            i2c.I2C_ReadREG(i2caddr, 0x02, i2cbuf, 2);      // get device ID
            SlotDevID[i] = (int)i2cbuf[0];
            i2c.I2C_ReadREG(i2caddr, 0x03, i2cbuf, 2);      // get firmware version
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
   
    for (i=0; i<=15; i++){
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
    char i2caddr;
    long slotn = std::strtol(slotnum.c_str(), NULL, 10);

    if ((slotn >=0) && (slotn < 16)){
        // execute only if a valid slot number range between 0 and 15
        char i2cbuf[2];
        i2caddr = (R1000_I2C_BASE + slotn) << 1;             // evaluate I2C address
        if (this->i2c.I2C_ReadREG(i2caddr, REG_TEMP, i2cbuf, 1) == 0){
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
