/* mbed Microcontroller Library - SerialHalfDuplex
 * Copyright (c) 2010-2011 ARM Limited. All rights reserved.
 */

#ifndef MBED_SERIALHALFDUPLEX_H
#define MBED_SERIALHALFDUPLEX_H

#include "device.h"

#if DEVICE_SERIAL

#include "Serial.h"
#include "PinNames.h"
#include "PeripheralNames.h"

namespace mbed {

/* Class: SerialHalfDuplex
 * A serial port (UART) for communication with other devices using  
 * Half-Duplex, allowing transmit and receive on a single
 * shared transmit and receive line. Only one end should be transmitting 
 * at a time.
 * 
 * Both the tx and rx pin should be defined, and wired together. 
 * This is in addition to them being wired to the other serial 
 * device to allow both read and write functions to operate.
 *
 *  Example:
 *  > // Send a byte to a second HalfDuplex device, and read the response
 *  >
 *  > #include "mbed.h"
 *  >
 *  > // p9 and p10 should be wired together to form "a"
 *  > // p28 and p27 should be wired together to form "b"
 *  > // p9/p10 should be wired to p28/p27 as the Half Duplex connection
 *  >
 *  > SerialHalfDuplex a(p9, p10);
 *  > SerialHalfDuplex b(p28, p27);
 *  >
 *  > void b_rx() { // second device response
 *  >     b.putc(b.getc() + 4);
 *  > }
 *  >   
 *  > int main() {
 *  >     b.attach(&b_rx);
 *  >     for(int c = 'A'; c < 'Z'; c++) {
 *  >         a.putc(c);
 *  >         printf("sent [%c]\n", c);
 *  >         wait(0.5);   // b should respond
 *  >         if(a.readable()) {
 *  >             printf("received [%c]\n", a.getc());
 *  >         }
 *  >     }
 *  > }
 * 
 * For Simplex and Full-Duplex Serial communication, see <Serial>
 */
class SerialHalfDuplex : public Serial {

public:
    /* Constructor: SerialHalfDuplex
     * Create a half-duplex serial port, connected to the specified transmit
     * and receive pins.
     *
     * These pins should be wired together, as well as to the target device
     *
     * Variables:
     *  tx - Transmit pin
     *  rx - Receive pin
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
     * Read a character from the serial port. This call will block
     * until a character is available. For testing if a character is
     * available for reading, see <readable>.
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

    virtual int _putc(int c);
    virtual int _getc(void);

}; // End class SerialHalfDuplex

} // End namespace

#endif

#endif
