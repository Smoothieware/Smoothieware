#ifndef R1000A_H
#define R1000A_H

#include "I2C.h" // mbed.h lib
//#include <string>

// R1000A class declaration goes here
class R1000A {
public:
    // Default Constructor
    R1000A();
    
    // Destructor
    ~R1000A();
    
    // Smoothie main module loading function
    void on_module_loaded();
    
    // Gcode received event
    void on_gcode_received(void *);
    
    // Scan I2C bus for modules
    void ScanI2CBus();
    
    // Report populated slot ID's
    void ReportI2CID();
    
    // I2C operations
    int I2C_ReadREG(char, char, char *, int);          // burst read
    int I2C_WriteREG(char, char, char *, int);         // burst write
    
    // other operations
    
    
private:
    // Member variables
    mbed::I2C* i2c;                 // i2c comm class
    int SlotDevID[16];              // board ID's for populated slots
    int SlotDevFW[16];              // device firmware version
};

#endif