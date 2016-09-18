#include "R1000A.h"

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


// command lookup table
const R1000A::ptentry_t R1000A::commands_table[] = {
    {"modtemp",       R1000A::modtemp},
    // unknown command
    {NULL, NULL}
};


R1000A::R1000A(){
    // Default Constructor
    //this->i2c = new R1000A_I2C;
}

R1000A::~R1000A(){
    // Destructor
}

void R1000A::on_module_loaded(){

    this->register_for_event(ON_GCODE_RECEIVED);        // Tell the kernel to call us whenever a gcode is received
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED); // register on console line received
    this->ScanI2CBus();                                 // perform initial I2C bus scan

    // FIXME implement MCU reset functions, add delays

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

void R1000A::on_gcode_received(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);     // Casting of the argument ( a Gcode object )
    THEKERNEL->streams->printf("R1000A: received line\r\n");
    if(gcode->has_m){
        unsigned int code =gcode->m;                  // store M command
        switch (code){
            case (1001):
                // M1000 command
                // just print out the command and arguments
                THEKERNEL->streams->printf("get_command  : %s\r\n",gcode->get_command());
                THEKERNEL->streams->printf("get_num_args : %d\r\n", gcode->get_num_args());
                THEKERNEL->streams->printf("g            : %d\r\n", gcode->g);
                THEKERNEL->streams->printf("m            : %d\r\n", gcode->m);
                THEKERNEL->streams->printf("is_error     : %d\r\n", gcode->is_error);

                // scan all I2C devices and report
                this->ScanI2CBus();
                this->ReportI2CID();
                break;
            default:
                break;
        }
    }
}

void R1000A::on_console_line_received(void* argument){
    SerialMessage new_message = *static_cast<SerialMessage *>(argument);
    string possible_command = new_message.message;
    string cmd = shift_parameter(possible_command);

    THEKERNEL->streams->printf("R1000: %s\r\n", possible_command.c_str());

    // static commands
    parse_command(cmd.c_str(), possible_command, new_message.stream);                   // parsing command against command table
}


void R1000A::ScanI2CBus(){
    // Scan addresses 0x10 through 0x1F
    // Address range is hardcoded to match R1000A platform
    // To identify which slot matches which I2C address use the following formula
    // Slot[n] has I2C address 0x10 + (n-1)
    // so Slot[1] has an I2C address of 0x10, Slot[2] is 0x11 ... Slot[16] is 0x1F
    
    char i2cbuf[3];     // create a 2 byte buffer for I2C
    int i;              // for loop variable
    char i2caddr;       // current I2C address
    
    for (i=0; i<=15; i++){
        i2caddr = (R1000_I2C_BASE + i) << 1;      // shift 1 to left to get 8-bit address
        // check for slave ack
        if (i2c.I2C_ReadREG(i2caddr, 0x01, i2cbuf, 1) == 0){
//            // continue reading from slave
//            if (i2cbuf[0] != 0x01){
//                // detected a device that doesn't belong to R1000A platform
//                SlotDevID[i-1] = -2;
//            }
//            else{
//                // detected a compatible R1000A device
//                I2C_ReadREG(i2caddr, 0x02, i2cbuf, 2);      // get device ID
//                SlotDevID[i-1] = i2cbuf[0];
//                I2C_ReadREG(i2caddr, 0x03, i2cbuf, 2);      // get firmware version
//                SlotDevFW[i-1] = i2cbuf[0];
//            }
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

bool R1000A::parse_command(const char *cmd, string args, StreamOutput *stream)
{
    for (const ptentry_t *p = commands_table; p->command != NULL; ++p) {
        if (strncasecmp(cmd, p->command, strlen(p->command)) == 0) {
            p->func(args, stream);
            return true;
        }
    }

    return false;
}

void R1000A::modtemp(string parameters, StreamOutput *stream ){
    // execute getmodtemp command
    // if there are no parameters
    if (!parameters.empty()) {
        // execute only if there are parameters
        string s = shift_parameter( parameters );
        long slotnum = std::strtol(s.c_str(), NULL, 10);
        char i2caddr;

        R1000A_I2C i2c;

        if ((slotnum >=0) && (slotnum < 16)){
            // execute only if a valid slot number range between 0 and 15
            char i2cbuf[2];
            i2caddr = (R1000_I2C_BASE + slotnum) << 1;             // evaluate I2C address
            if (i2c.I2C_ReadREG(i2caddr, REG_TEMP, i2cbuf, 1) == 0){
                // execute only if reading operation is successful
                THEKERNEL->streams->printf("Slot %lu Temp : %d\r\n", slotnum, i2cbuf[0]);
            }
            else
            {
                // output an error message
                THEKERNEL->streams->printf("Slot %lu did not ack!\r\n", slotnum);
            }
            i2c.~R1000A_I2C();              // destructor
        }
        else{
            THEKERNEL->streams->printf("Invalid slot %lu\r\n", slotnum);
        }

    }
}
