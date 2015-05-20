
/**
 * @file    BufferedSoftSerial.h
 * @brief   Software Buffer - Extends mbed Serial functionallity adding irq driven TX and RX
 * @author  sam grove
 * @version 1.0
 * @see     
 *
 * Copyright (c) 2013
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BUFFEREDSOFTSERIAL_H
#define BUFFEREDSOFTSERIAL_H
 
#include "mbed.h"
#include "Buffer.h"
#include "SoftSerial.h"

/** A serial port (UART) for communication with other serial devices
 *
 * Can be used for Full Duplex communication, or Simplex by specifying
 * one pin as NC (Not Connected)
 *
 * This uses software serial emulation, regular serial pins are alot better,
 * however if you don't have spare ones, you can use this. It is advicable
 * to put the serial connection with highest speed to hardware serial.
 *
 * If you lack RAM memory you can also use SoftSerial without this buffer around it.
 * In that case it is fully blocking.
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "BufferedSoftSerial.h"
 * 
 * SoftSerial block(USBTX, USBRX);
* BufferedSoftSerial buf(USBTX, USBRX);
 * 
 * int main()
 * {
 *     while(1) {
 *         Timer s;
 * 
 *         s.start();
 *         buf.printf("Hello World - buffered\r\n");
 *         int buffered_time = s.read_us();
 *         wait(0.1f); // give time for the buffer to empty
 * 
 *         s.reset();
 *         block.printf("Hello World - blocking\r\n");
 *         int polled_time = s.read_us();
 *         s.stop();
 *         wait(0.1f); // give time for the buffer to empty
 * 
 *         buf.printf("printf buffered took %d us\r\n", buffered_time);
 *         buf.printf("printf blocking took %d us\r\n", polled_time);
 *         wait(5);
 *     }
 * }
 * @endcode
 */

/**
 *  @class BufferedSerial
 *  @brief Software buffers and interrupt driven tx and rx for SoftSerial
 */  
class BufferedSoftSerial : public SoftSerial 
{
private:
    Buffer <char> _rxbuf;
    Buffer <char> _txbuf;
 
    void rxIrq(void);
    void txIrq(void);
    void prime(void);
    
public:
    /** Create a BufferedSoftSerial port, connected to the specified transmit and receive pins
     *  @param tx Transmit pin
     *  @param rx Receive pin
     *  @note Either tx or rx may be specified as NC if unused
     */
    BufferedSoftSerial(PinName tx, PinName rx, const char* name=NULL);
    
    /** Check on how many bytes are in the rx buffer
     *  @return 1 if something exists, 0 otherwise
     */
    virtual int readable(void);
    
    /** Check to see if the tx buffer has room
     *  @return 1 always has room and can overwrite previous content if too small / slow
     */
    virtual int writeable(void);
    
    /** Get a single byte from the BufferedSoftSerial Port.
     *  Should check readable() before calling this.
     *  @return A byte that came in on the BufferedSoftSerial Port
     */
    virtual int getc(void);
    
    /** Write a single byte to the BufferedSoftSerial Port.
     *  @param c The byte to write to the BufferedSoftSerial Port
     *  @return The byte that was written to the BufferedSoftSerial Port Buffer
     */
    virtual int putc(int c);
    
    /** Write a string to the BufferedSoftSerial Port. Must be NULL terminated
     *  @param s The string to write to the Serial Port
     *  @return The number of bytes written to the Serial Port Buffer
     */
    virtual int puts(const char *s);
    
    /** Write a formatted string to the BufferedSoftSerial Port.
     *  @param format The string + format specifiers to write to the BufferedSoftSerial Port
     *  @return The number of bytes written to the Serial Port Buffer
     */
    virtual int printf(const char* format, ...);
    
    /** Write data to the BufferedSoftSerial Port
     *  @param s A pointer to data to send
     *  @param length The amount of data being pointed to
     *  @return The number of bytes written to the Serial Port Buffer
     */
    virtual ssize_t write(const void *s, std::size_t length);
};

#endif
