#ifndef R1000A_H
#define R1000A_H

#include "modules/JuicyBoard/R1000A_I2C/R1000A_I2C.h"

#include "Pin.h"            // I/O
#include "I2C.h"            // mbed.h lib
#include "Module.h"         // for registering events
#include <string>
using std::string;

#define REG_TEMP            0x10        // I2C register address of temperature readback
#define RESET_DELAY         50          // 100ms module reset delay
#define RES_CH1             0.005       // channel 1 sense resistor value, from board hardware
#define RES_CH2             0.05        // channel 2 sense resistor value, from board hardware
#define RES_CH3             0.05        // channel 3 sense resistor value, from board hardware

#define EE_TW               5           // EEPROM write time in ms

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

        // power monitor converging functions
        float evalCURR(char *);                 // converts I2C shunt reading to mV
        float evalVOLT(char *);                 // converts I2C bus reading to V

        // eeprom private functions
        char readEEbyte(unsigned int);          // read a single byte from EEPROM
        char writeEEbyte(unsigned int,char);    // write a single byte from EEPROM

        // hex file private functions
        void dumphex(const char *);                   // analyzes hex file, assumes
};

#endif
