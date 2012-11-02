/* mbed Microcontroller Library - I2C
 * Copyright (c) 2007-2011 ARM Limited. All rights reserved.
 */ 
 
#ifndef MBED_I2C_H
#define MBED_I2C_H

#include "device.h"

#if DEVICE_I2C

#include "platform.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "Base.h"

namespace mbed {

/* Class: I2C
 *  An I2C Master, used for communicating with I2C slave devices
 *
 * Example:
 * > // Read from I2C slave at address 0x62
 * >
 * > #include "mbed.h"
 * >
 * > I2C i2c(p28, p27);
 * >
 * > int main() {
 * >     int address = 0x62;
 * >     char data[2];
 * >     i2c.read(address, data, 2);
 * > }
 */
class I2C : public Base {

public:

    enum RxStatus {
        NoData
        , MasterGeneralCall
        , MasterWrite
        , MasterRead
    };

    enum Acknowledge {
          NoACK = 0
        , ACK   = 1
    };

    /* Constructor: I2C
     *  Create an I2C Master interface, connected to the specified pins
     *
     * Variables:
     *  sda - I2C data line pin
     *  scl - I2C clock line pin
     */
    I2C(PinName sda, PinName scl, const char *name = NULL);

    /* Function: frequency
     *  Set the frequency of the I2C interface
     *
     * Variables:
     *  hz - The bus frequency in hertz
     */
    void frequency(int hz);

    /* Function: read
     *  Read from an I2C slave
     *
     * Performs a complete read transaction. The bottom bit of
     * the address is forced to 1 to indicate a read.
     *
     * Variables:
     *  address - 8-bit I2C slave address [ addr | 1 ]
     *  data - Pointer to the byte-array to read data in to 
     *  length - Number of bytes to read
     *  repeated - Repeated start, true - don't send stop at end
     *  returns - 0 on success (ack), or non-0 on failure (nack)
     */ 
    int read(int address, char *data, int length, bool repeated = false); 

    /* Function: read
     *  Read a single byte from the I2C bus
     *
     * Variables:
     *  ack - indicates if the byte is to be acknowledged (1 = acknowledge)
     *  returns - the byte read
     */
    int read(int ack);

    /* Function: write
     *  Write to an I2C slave
     *
     * Performs a complete write transaction. The bottom bit of
     * the address is forced to 0 to indicate a write.
     *
     * Variables:
     *  address - 8-bit I2C slave address [ addr | 0 ]
     *  data - Pointer to the byte-array data to send 
     *  length - Number of bytes to send
     *  repeated - Repeated start, true - do not send stop at end
     *  returns - 0 on success (ack), or non-0 on failure (nack)
     */ 
    int write(int address, const char *data, int length, bool repeated = false);

    /* Function: write
     *  Write single byte out on the I2C bus
     *
     * Variables:
     *  data - data to write out on bus
     *  returns - a '1' if an ACK was received, a '0' otherwise
     */
    int write(int data);

    /* Function: start
     *  Creates a start condition on the I2C bus
     */

    void start(void);

    /* Function: stop
     *  Creates a stop condition on the I2C bus
     */
    void stop(void);

protected:

    void aquire();

    I2CName     _i2c;
    static I2C  *_owner;
    int         _hz;

};

} // namespace mbed

#endif

#endif

