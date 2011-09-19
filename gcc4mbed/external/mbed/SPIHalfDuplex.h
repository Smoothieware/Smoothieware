/* mbed Microcontroller Library - SPI
 * Copyright (c) 2010 ARM Limited. All rights reserved.
 * jward
 */

#ifndef MBED_SPIHALFDUPLEX_H
#define MBED_SPIHALFDUPLEX_H

#include "SPI.h"

namespace mbed {

/* Class: SPIHalfDuplex
 *   A SPI half-duplex master, used for communicating with SPI slave devices
 * over a shared data line.
 *
 * The default format is set to 8-bits for both master and slave, and a
 * clock frequency of 1MHz
 *
 * Most SPI devies will also require Chip Select and Reset signals. These
 * can be controlled using <DigitalOut> pins.
 *
 * Although this is for a shared data line, both MISO and MOSI are defined,
 * and should be tied together externally to the mbed. This class handles
 * the tri-stating of the MOSI pin.
 *
 * Example:
 * > // Send a byte to a SPI half-duplex slave, and record the response
 * >
 * > #include "mbed.h"
 * > 
 * > SPIHalfDuplex device(p5, p6, p7) // mosi, miso, sclk
 * >
 * > int main() {
 * >     int respone = device.write(0xAA);
 * > }
 */

class SPIHalfDuplex : public SPI {

public:
    
    /* Constructor: SPIHalfDuplex
     *  Create a SPI half-duplex master connected to the specified pins
     *
     * Variables:
     *  mosi - SPI Master Out, Slave In pin
     *  miso - SPI Master In, Slave Out pin
     *  sclk - SPI Clock pin
     *  name - (optional) A string to identify the object
     *
     * Pin Options:
     *  (5, 6, 7) or (11, 12, 13)
     *
     *  mosi or miso can be specfied as NC if not used
     */
    SPIHalfDuplex(PinName mosi, PinName miso, PinName sclk,
        const char *name = NULL);

#if 0 // Inherited from SPI - documentation only
    /* Function: format
     *  Configure the data transmission format
     *
     * Variables:
     *  bits - Number of bits per SPI frame (4 - 16)
     *  mode - Clock polarity and phase mode (0 - 3)
     *
     * > mode | POL PHA
     * > -----+--------
     * >   0  |  0   0
     * >   1  |  0   1
     * >   2  |  1   0
     * >   3  |  1   1
     */
    void format(int bits, int mode = 0);

    /* Function: frequency
     *  Set the spi bus clock frequency
     *
     * Variables:
     *  hz - SCLK frequency in hz (default = 1MHz)
     */
    void frequency(int hz = 1000000);
#endif

    /* Function: write
     *  Write to the SPI Slave and return the response
     *
     * Variables:
     *  value - Data to be sent to the SPI slave
     *  returns - Response from the SPI slave
     */
    virtual int write(int value);
    
    /* Function: slave_format
     *  Set the number of databits expected from the slave, from 4-16
     *
     * Variables:
     *  sbits - Number of expected bits in the slave response
     */
    void slave_format(int sbits);

protected:

    PinName _mosi;
    PinName _miso;
    int     _sbits;

}; // End of class

} // End of namespace mbed

#endif
