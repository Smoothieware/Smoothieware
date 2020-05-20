/* 
 * SoftSerial by Erik Olieman
 * Date: 05 Jul 2014
 * Revision: 10:236fce2e5b8c
 * URL: http://developer.mbed.org/users/Sissors/code/SoftSerial/
 */

#include "libs/Kernel.h"
#include "SoftSerial.h"


uint32_t overhead_us = 200 * 1000000 / SystemCoreClock;         //Random estimation of the overhead of mbed libs, makes slow devices like LPC812 @ 12MHz perform better

int SoftSerial::_getc( void ) {
    out_valid = false;
    return out_buffer;
}

int SoftSerial::readable(void) {
    return out_valid;
}

//Start receiving byte
void SoftSerial::rx_gpio_irq_handler(void) {
    rxticker.prime();
    rxticker.setNext(bit_period + (bit_period >> 1) - overhead_us);
    rx->fall(NULL);
    rx_bit = 0;
    rx_error = false;
};    

void SoftSerial::rx_handler(void) {
    //Receive data
    int val = rx->read();
 
    rxticker.setNext(bit_period);
    rx_bit++;
    
    
    if (rx_bit <= _bits) {
        read_buffer |= val << (rx_bit - 1);
        return;
    }
    
    //Receive parity
    bool parity_count;
    if (rx_bit == _bits + 1) {
        switch (_parity) {
            case Forced1:
                if (val == 0)
                    rx_error = true;
                return;
            case Forced0:
                if (val == 1)
                    rx_error = true;
                return;
            case Even:
            case Odd:
                parity_count = val;
                for (int i = 0; i<_bits; i++) {
                    if (((read_buffer >> i) & 0x01) == 1)
                        parity_count = !parity_count;
                }
                if ((parity_count) && (_parity == Even))
                    rx_error = true;
                if ((!parity_count) && (_parity == Odd))
                    rx_error = true;
                return;
            case None:
                // No parity, nothing to do here
                break;
        }
    }
    
    //Receive stop
    if (rx_bit < _bits + (bool)_parity + _stop_bits) {
        if (!val)
            rx_error = true;
        return;
    }    
    
    //The last stop bit
    if (!val)
        rx_error = true;
    
    if (!rx_error) {
        out_valid = true;
        out_buffer = read_buffer;
        fpointer[RxIrq].call();
    }
    read_buffer = 0;
    rxticker.detach(); 
    rx->fall(this, &SoftSerial::rx_gpio_irq_handler);
}

