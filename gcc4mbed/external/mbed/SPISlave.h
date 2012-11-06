/* mbed Microcontroller Library - SPISlave
 * Copyright (c) 2010-2011 ARM Limited. All rights reserved. 
 */

#ifndef MBED_SPISLAVE_H
#define MBED_SPISLAVE_H

#include "device.h"

#if DEVICE_SPISLAVE

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: SPISlave
 *  A SPI slave, used for communicating with a SPI Master device
 *
 * The default format is set to 8-bits, mode 0, and a clock frequency of 1MHz
 *
 * Example:
 * > // Reply to a SPI master as slave
 * >
 * > #include "mbed.h"
 * >
 * > SPISlave device(p5, p6, p7, p8); // mosi, miso, sclk, ssel
 * >
 * > int main() {
 * >     device.reply(0x00);              // Prime SPI with first reply
 * >     while(1) {
 * >         if(device.receive()) {
 * >             int v = device.read();   // Read byte from master
 * >             v = (v + 1) % 0x100;     // Add one to it, modulo 256
 * >             device.reply(v);         // Make this the next reply
 * >         }
 * >     }
 * > }
 */ 
class SPISlave : public Base {

public:

    /* Constructor: SPI
     *  Create a SPI slave connected to the specified pins
     *
     * Variables:
     *  mosi - SPI Master Out, Slave In pin
     *  miso - SPI Master In, Slave Out pin
     *  sclk - SPI Clock pin
     *  ssel - SPI chip select pin
     *  name - (optional) A string to identify the object     
     *
     * Pin Options:
     *  (5, 6, 7i, 8) or (11, 12, 13, 14)
     *
     *  mosi or miso can be specfied as NC if not used
     */
    SPISlave(PinName mosi, PinName miso, PinName sclk, PinName ssel,
        const char *name = NULL);

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

    /* Function: receive
     *  Polls the SPI to see if data has been received
     *
     * Variables:
     *  returns - zero if no data, 1 otherwise
     */
    int receive(void);

    /* Function: read
     *  Retrieve  data from receive buffer as slave
     *
     * Variables:
     *  returns - the data in the receive buffer
     */
    int read(void);

    /* Function: reply
     *  Fill the transmission buffer with the value to be written out
     *  as slave on the next received message from the master.
     *
     * Variables:
     *  value - the data to be transmitted next
     */
    void reply(int value);

protected:

	SPIName _spi;
	
    int _bits;
    int _mode;
    int _hz;

};

} // namespace mbed

#endif

#endif
