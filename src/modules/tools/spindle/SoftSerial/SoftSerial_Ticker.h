/* 
 * SoftSerial by Erik Olieman
 * Date: 05 Jul 2014
 * Revision: 10:236fce2e5b8c
 * URL: http://developer.mbed.org/users/Sissors/code/SoftSerial/
 */

//A modified version of the regular ticker/timeout libraries to allow us to do timeout without losing accuracy

#ifndef FLEXTICKER_H
#define FLEXTICKER_H

#include "mbed.h"

class FlexTicker: public TimerEvent {
    public:
    template<typename T>
    void attach(T* tptr, void (T::*mptr)(void)) {
        _function.attach(tptr, mptr);
    }
 
    /** Detach the function
     */
    void detach() {
        remove();
    }
    
    void setNext(int delay) {
        insert(event.timestamp + delay);
    }
    
    void prime(void) {
        event.timestamp = us_ticker_read();
    }
 
protected:
    virtual void handler() {
        _function.call();
    }
 
    unsigned int _delay;
    FunctionPointer _function;
};

#endif
