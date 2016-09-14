#include "R1000A.h"
//#include <string>
//using std::string;

#include "StreamOutputPool.h"

#include "CurrentControl.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"

#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"

// define configuration checksums here


R1000A::R1000A(){
    // Default Constructor
    this->i2c = new mbed::I2C(P0_27, P0_28);    // define master i2c comm class
    this->i2c->frequency(100000);               // set I2C bus freq in Hz
}

R1000A::~R1000A(){
    // Destructor
}

void R1000A::on_module_loaded(){
    // example of config checks on class loading
    // see which chip to use
//    int chip_checksum = get_checksum(THEKERNEL->config->value(digipotchip_checksum)->by_default("mcp4451")->as_string());
//    if(chip_checksum == mcp4451_checksum) {
//        digipot = new MCP4451();
//    } else { // need a default so use smoothie
//        digipot = new MCP4451();
//    }

    // instantiate a new i2c class here
    

//    digipot->set_max_current( THEKERNEL->config->value(digipot_max_current )->by_default(2.0f)->as_number());
//    digipot->set_factor( THEKERNEL->config->value(digipot_factor )->by_default(113.33f)->as_number());

    // Get configuration
//    this->digipot->set_current(0, THEKERNEL->config->value(alpha_current_checksum  )->by_default(0.8f)->as_number());
//    this->digipot->set_current(1, THEKERNEL->config->value(beta_current_checksum   )->by_default(0.8f)->as_number());
//    this->digipot->set_current(2, THEKERNEL->config->value(gamma_current_checksum  )->by_default(0.8f)->as_number());
//    this->digipot->set_current(3, THEKERNEL->config->value(delta_current_checksum  )->by_default(0.8f)->as_number());
//    this->digipot->set_current(4, THEKERNEL->config->value(epsilon_current_checksum)->by_default(-1)->as_number());
//    this->digipot->set_current(5, THEKERNEL->config->value(zeta_current_checksum   )->by_default(-1)->as_number());
//    this->digipot->set_current(6, THEKERNEL->config->value(eta_current_checksum    )->by_default(-1)->as_number());
//    this->digipot->set_current(7, THEKERNEL->config->value(theta_current_checksum  )->by_default(-1)->as_number());

//    int i;    // for loop variable
//    for (i=0x10;i<=0x1F;i++)
//    {
//        this->digipot->get_i2c_id(i << 1);
//    }
    
//    int i;
//    for (i=0;i<100;i++)        // repeat for 1000 times for test
//    {
//        this->digipot->get_i2c_id(0x18 << 1);
//        this->digipot->get_i2c_temperature(0x18 << 1);
//        wait_ms(10);
//    }
    
//    this->register_for_event(ON_GCODE_RECEIVED);


}

void R1000A::on_gcode_received(void *){
    // do nothing
}

void R1000A::ScanI2CBus(){
    // Scan addresses 0x10 through 0x1F
    // Address range is hardcoded to match R1000A platform
    // To identify which slot matches which I2C address use the following formula
    // Slot[n] has I2C address 0x10 + (n-1)
    // so Slot[1] has an I2C address of 0x10, Slot[2] is 0x11 ... Slot[16] is 0x1F
    
    char i2cbuf[3];     // create a 2 byte buffer for I2C
    char i;             // 8-bit for loop variable
    char i2caddr;       // current I2C address
    
    for (i=1; i<=16; i++){
        i2caddr = 0x10 + i - 1;
        // check for slave ack
        if (I2C_ReadREG(i2caddr, 0x01, i2cbuf, 1) == 0){
            // continue reading from slave
            if (i2cbuf[0] != 0x01){
                // detected a device that doesn't belong to R1000A platform
                SlotDevID[i-1] = -2;
            }
            else{
                // detected a compatible R1000A device
                I2C_ReadREG(i2caddr, 0x02, i2cbuf, 2);
                SlotDevID[i-1] = i2cbuf[0];
                SlotDevFW[i-1] = i2cbuf[1];
            }
        }
        else{
            SlotDevID[i-1] = -1;
        }
    } 
}

void R1000A::ReportI2CID(){
    char i;                     // 8-bit for loop variable
   
    for (i=1; i<=16; i++){
        if (SlotDevID[i-1] == -1){
            THEKERNEL->streams->printf("Slot %d NO CARD\r\n", i);
        }
        else if (SlotDevID[i-1] == -2){
            THEKERNEL->streams->printf("Slot %d NOT JuicyBoard COMPATIBLE!\r\n", i);
        }
        else{
            THEKERNEL->streams->printf("Slot %d MOD #%d, FW %d\r\n", i, SlotDevID[i-1], SlotDevFW[i-1]);
        }
    }
}

int R1000A::I2C_ReadREG(char I2CAddr, char REGAddr, char * data, int length){
    // perform burst register read
    int i;      // for loop variable
    // set the register to access
    this->i2c->start();
    if (this->i2c->write(I2CAddr) != 0){		// check for slave ack
        // slave I2C is not acknowledging, exit function
        this->i2c->stop();
        return -1;
    }
    this->i2c->write(REGAddr);                  // register address
    this->i2c->stop();

    // read part
    this->i2c->start();
    this->i2c->write(I2CAddr);                  // slave I2C address
    for (i=0; i<length; i++){                   // loop over every byte
        data[i] = this->i2c->read(1);        
    }
    this->i2c->read(0);                         // extra dummy read for mbed I2C to stop properly
    this->i2c->stop();
    return 0;
}

int R1000A::I2C_WriteREG(char I2CAddr, char REGAddr, char * data, int length)
{
    // perform burst register read
    int i;      // for loop variable
    
    // set the register to access
    this->i2c->start();
    if (this->i2c->write(I2CAddr) != 0){        // check for slave ack
        // slave I2C is not acknowledging, exit function
        this->i2c->stop();
        return -1;
    }
    this->i2c->write(REGAddr);                  // register address
    for (i=0; i<length; i++){
        this->i2c->write(data[i]);              // write data one by one
    }
    this->i2c->stop();
    return 0;
}
