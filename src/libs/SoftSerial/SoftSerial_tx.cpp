/* 
 * SoftSerial by Erik Olieman
 * Date: 05 Jul 2014
 * Revision: 10:236fce2e5b8c
 * URL: http://developer.mbed.org/users/Sissors/code/SoftSerial/
 */

#include "SoftSerial.h"

int SoftSerial::_putc(int c)
{
    while(!writeable());
    prepare_tx(c);
    tx_bit = 0;
    txticker.prime();
    tx_handler();
    return 0;
}

void SoftSerial::send_break(void) {
    while(!writeable());
    tx_bit = 0;         //Just to make sure it appears as non-writable to other threads/IRQs
    tx->write(0);
    wait_us((bit_period * _total_bits * 3) / 2);
    tx->write(1);
    tx_bit = -1;
}

int SoftSerial::writeable(void)
{
    if (!tx_en)
        return false;
    if (tx_bit == -1)
        return true;
    return false;
}

void SoftSerial::tx_handler(void)
{
    if (tx_bit == _total_bits) {
        tx_bit = -1;
        fpointer[TxIrq].call();
        return;
    }

    //Flip output
    int cur_out = tx->read();
    tx->write(!cur_out);

    //Calculate when to do it again
    int count = bit_period;
    tx_bit++;
    while(((_char >> tx_bit) & 0x01) == !cur_out) {
        count+=bit_period;
        tx_bit++;
    }

    txticker.setNext(count);
}

void SoftSerial::prepare_tx(int c)
{
    _char = c << 1;

    bool parity;
    switch (_parity) {
        case Forced1:
            _char |= 1 << (_bits + 1);
        case Even:
            parity = false;
            for (int i = 0; i<_bits; i++) {
                if (((_char >> i) & 0x01) == 1)
                    parity = !parity;
            }
            _char |= parity << (_bits + 1);
        case Odd:
            parity = true;
            for (int i = 0; i<_bits; i++) {
                if (((_char >> i) & 0x01) == 1)
                    parity = !parity;
            }
            _char |= parity << (_bits + 1);
    }
    
    _char |= 0xFFFF << (1 + _bits + (bool)_parity);
    _char &= ~(1<<_total_bits);
}
