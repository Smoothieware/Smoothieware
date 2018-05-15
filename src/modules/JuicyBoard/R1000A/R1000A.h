#ifndef R1000A_H
#define R1000A_H


#include "Pin.h"            // I/O
#include "Module.h"         // for registering events
#include <string>
using std::string;

#define REG_TEMP            0x10        // I2C register address of temperature readback
#define RESET_DELAY         10          // 100ms module reset delay
#define RES_CH1             0.005       // channel 1 sense resistor value, from board hardware
#define RES_CH2             0.05        // channel 2 sense resistor value, from board hardware
#define RES_CH3             0.05        // channel 3 sense resistor value, from board hardware

#define EE_TW               5           // EEPROM write time in ms

// ---------------------------------
// Boot Loader Command Codes
// ---------------------------------
#define TGT_CMD_RESET_MCU           0x00
#define TGT_CMD_GET_VERSION         0x01
#define TGT_CMD_ERASE_FLASH_PAGE    0x05
#define TGT_CMD_WRITE_FLASH_BYTES   0x06
#define TGT_CMD_READ_FLASH_BYTES    0x07
#define TGT_CMD_CHECK_SIG           0x08
#define TGT_CMD_START_APPFW         0x0C

// bootloader defines
#define TGT_CMD_CHECK_BLSTAT        0x0B    // command to check bootloader status
#define TGT_RSP_BL_MODE             0x03

// ---------------------------------
// Target BL Response Codes
// ---------------------------------
#define TGT_RSP_OK                  0x00 // RSP_OK should always be 0
#define TGT_RSP_PARAMETER_INVALID   0x01
#define TGT_RSP_UNSUPPORTED_CMD     0xFE
#define TGT_RSP_BL_MODE             0x03
#define TGT_RSP_ERROR               0x80
#define TGT_RSP_ADDR_INVALID        0x81 // added by Juicyboard, invalid address to flash, write or read

// Module Flash Address Range
#define MOD_FADDR_START             0x0
#define MOD_PAGE_SIZE               512
#define MOD_FADDR_END               0x1DFF

class StreamOutput;

// R1000A class declaration goes here
class R1000A : public Module {
    public:
        // Default Constructor
        R1000A();

        // Smoothie main module loading function
        void on_module_loaded();

        // console line received
        void on_console_line_received(void *);

        // Scan I2C bus for modules
        void ScanI2CBus();

        // Activate populated modules
        void ActivateModules();

        // Report populated slot ID's
        void ReportI2CID();
    
        // reports temperature of card on slot
        int getTemp(int Slot);

        // Accessor functions
        int getSlotDevID(int) const;

        // reset all modules
        void ResetMods(void);

        // power monitor functions
        void InitPowerMon(void);
        void getPowerMonCfg(void);
        void readPowerMon(void);

        // EEPROM functions


    private:
        // Member variables
        Pin *ModResetPin;               // define reset pin

        int SlotPlatID[16];             // module platform ID
        int SlotDevID[16];              // module device ID
        int SlotDevFW[16];              // module firmware version

        // console commands
        void getTemp(string);
        void i2creaderr(void);          // spits out I2C read error message

        // power monitor conversion functions
        float evalCURR(char *);                 // converts I2C shunt reading to mV
        float evalVOLT(char *);                 // converts I2C bus reading to V

        // eeprom private functions
        char readEEbyte(unsigned int);          // read a single byte from EEPROM
        char writeEEbyte(unsigned int,char);    // write a single byte from EEPROM

        // hex file private functions
        void wrhex2bl(const char *, int);       // write hex file to bootloader
        void dumphex(int);                      // dumps module flash memory to console
        unsigned char retcharval(char,char);    // returns a char (numerical byte value) of given upper and lower characters
        int checkfwsig(int);                    // checks if application firmware signature is correct
};

#endif
