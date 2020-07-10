/* 
 * SoftSerial by Erik Olieman
 * Date: 05 Jul 2014
 * Revision: 10:236fce2e5b8c
 * URL: http://developer.mbed.org/users/Sissors/code/SoftSerial/
 */

#ifndef SOFTSERIAL_H
#define SOFTSERIAL_H

#include "mbed.h"
#include "SoftSerial_Ticker.h"
/** A software serial implementation
 *
 */
class SoftSerial: public Stream {

public:
    /**
    * Constructor
    *
    * @param TX Name of the TX pin, NC for not connected
    * @param RX Name of the RX pin, NC for not connected, must be capable of being InterruptIn
    * @param name Name of the connection
    */
    SoftSerial(PinName TX, PinName RX, const char* name = NULL);
    virtual ~SoftSerial();
    
    /** Set the baud rate of the serial port
     *
     *  @param baudrate The baudrate of the serial port (default = 9600).
     */
    void baud(int baudrate);

    enum Parity {
        None = 0,
        Odd,
        Even,
        Forced1,
        Forced0
    };

    enum IrqType {
        RxIrq = 0,
        TxIrq
    };

    /** Set the transmission format used by the serial port
     *
     *  @param bits The number of bits in a word (default = 8)
     *  @param parity The parity used (SerialBase::None, SerialBase::Odd, SerialBase::Even, SerialBase::Forced1, SerialBase::Forced0; default = SerialBase::None)
     *  @param stop The number of stop bits (default = 1)
     */
    void format(int bits=8, Parity parity=SoftSerial::None, int stop_bits=1);

    /** Determine if there is a character available to read
     *
     *  @returns
     *    1 if there is a character available to read,
     *    0 otherwise
     */
    int readable();

    /** Determine if there is space available to write a character
     *
     *  @returns
     *    1 if there is space to write a character,
     *    0 otherwise
     */
    int writeable();

    /** Attach a function to call whenever a serial interrupt is generated
     *
     *  @param fptr A pointer to a void function, or 0 to set as none
     *  @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    void attach(void (*fptr)(void), IrqType type=RxIrq) {
        fpointer[type].attach(fptr);
    }

    /** Attach a member function to call whenever a serial interrupt is generated
     *
     *  @param tptr pointer to the object to call the member function on
     *  @param mptr pointer to the member function to be called
     *  @param type Which serial interrupt to attach the member function to (Seriall::RxIrq for receive, TxIrq for transmit buffer empty)
     */
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void), IrqType type=RxIrq) {
        fpointer[type].attach(tptr, mptr);
    }

protected:
    DigitalOut *tx;
    InterruptIn *rx;
    
    bool tx_en, rx_en;
    int bit_period;
    int _bits, _stop_bits, _total_bits;
    Parity _parity;
    
    FunctionPointer fpointer[2];
    
    //rx
    void rx_gpio_irq_handler(void);
    void rx_handler(void);
    int read_buffer, rx_bit;
    volatile int out_buffer;
    volatile bool out_valid;
    bool rx_error;
    FlexTicker rxticker;
    
    //tx
    void tx_handler(void);
    void prepare_tx(int c);
    FlexTicker txticker;
    int _char;
    volatile int tx_bit;
    
    
    
    virtual int _getc();
    virtual int _putc(int c);
};


#endif

