/* mbed Microcontroller Library - SerialHalfDuplex
 * Copyright (c) 2010 ARM Limited. All rights reserved.
 * jward
 */

#ifndef MBED_SERIALHALFDUPLEX_H
#define MBED_SERIALHALFDUPLEX_H

#include "Serial.h"
#include "PinNames.h"
#include "PeripheralNames.h"

namespace mbed {

/* Class: SerialHalfDuplex
 *  A serial port (UART) for communication with other devices, with a single
 *  shared transmit and receive line.
 *
 *  If the device both transmits and receives, then both (separate) pins need
 *  to be defined, and tied together externally.
 *
 *  Example:
 *  > // Send a byte as a master, and receive a byte as a slave
 *  >
 *  > #include "mbed.h"
 *  >
 *  > SerialHalfDuplex master(p9, p10);
 *  >
 *  > int main() {
 *  >     int outbyte = master.putc(0x55);
 *  >     int retbyte = master.getc();
 *  >     printf("Wrote: %02X  Read: %02X\n", outbyte, retbyte);
 *  > }
 */
class SerialHalfDuplex : public Serial {

public:
    /* Constructor: SerialHalfDuplex
     * Create a half-duplex serial port, connected to the specified transmit
     * and receive pins.
     *
     * Variables:
     *  tx - Transmit pin
     *  rx - Receive pin
     *
     *  Note: Either tx or rx may be specified as NC if unused
     */

    SerialHalfDuplex(PinName tx, PinName rx, const char *name = NULL);

#if 0       // Inherited from Serial class, for documentation
    /* Function: baud
     *  Set the baud rate of the serial port
     *
     * Variables:
     *  baudrate - The baudrate of the serial port (default = 9600).
     */
    void baud(int baudrate);

    enum Parity {
        None = 0
        , Odd
        , Even
        , Forced1
        , Forced0
    };

    /* Function: format
     *  Set the transmission format used by the Serial port
     *
     * Variables:
     *  bits - The number of bits in a word (5-8; default = 8)
     *  parity - The parity used (Serial::None, Serial::Odd, 
Serial::Even, Serial::Forced1, Serial::Forced0; default = Serial::None)
     *  stop - The number of stop bits (1 or 2; default = 1)
     */
    void format(int bits = 8, Parity parity = Serial::None, int stop_bits 
= 1);

    /* Function: putc
     *  Write a character
     *
     * Variables:
     *  c - The character to write to the serial port
     */
    int putc(int c);

    /* Function: getc
     *  Read a character
     *
     * Variables:
     *  returns - The character read from the serial port
     */
    int getc();

    /* Function: printf
     *  Write a formated string
     *
     * Variables:
     *  format - A printf-style format string, followed by the
     *      variables to use in formating the string.
     */
    int printf(const char* format, ...);

    /* Function: scanf
     *  Read a formated string
     *
     * Variables:
     *  format - A scanf-style format string,
     *      followed by the pointers to variables to store the results.
     */
    int scanf(const char* format, ...);

    /* Function: readable
     *  Determine if there is a character available to read
     *
     * Variables:
     *  returns - 1 if there is a character available to read, else 0
     */
    int readable();

    /* Function: writeable
     *  Determine if there is space available to write a character
     *
     * Variables:
     *  returns - 1 if there is space to write a character, else 0
     */
    int writeable();

    /* Function: attach
     *  Attach a function to call whenever a serial interrupt is generated
     *
     * Variables:
     *  fptr - A pointer to a void function, or 0 to set as none
     */
    void attach(void (*fptr)(void));

    /* Function: attach
     *  Attach a member function to call whenever a serial interrupt is generated
     *
     * Variables:
     *  tptr - pointer to the object to call the member function on
     *  mptr - pointer to the member function to be called
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void));

#endif

protected:
    PinName     _txpin;
    int         _pinfunc;

    virtual int _putc(int c);
    virtual int _getc(void);

}; // End class SerialHalfDuplex

} // End namespace

#endif
