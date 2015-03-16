#ifndef PIN_H
#define PIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "mbed.h"

#include "PinNames.h"

namespace mbed {
    class PwmOut;
}

class Pin {
    public:
        Pin();

        Pin* from_string(std::string value);
        /* FIXME STM32 */
        inline bool connected(){
            //~ return this->valid;
            return true;
        }

        inline bool equals(const Pin& other) const {
            //~ return (this->pin == other.pin) && (this->port == other.port);
            return true;
        }

        inline Pin* as_output(){
            //~ if (this->valid)
                //~ this->port->FIODIR |= 1<<this->pin;
            return this;
        }

        inline Pin* as_input(){
            //~ if (this->valid)
                //~ this->port->FIODIR &= ~(1<<this->pin);
            return this;
        }

        Pin* as_open_drain(void);

        Pin* as_repeater(void);

        Pin* pull_up(void);

        Pin* pull_down(void);

        Pin* pull_none(void);

        inline bool get(){
            //~ if (!this->valid) return false;
            //~ return this->inverting ^ (( this->port->FIOPIN >> this->pin ) & 1);
            return true;
        }

        inline void set(bool value)
        {
            //~ if (!this->valid) return;
            //~ if ( this->inverting ^ value )
                //~ this->port->FIOSET = 1 << this->pin;
            //~ else
                //~ this->port->FIOCLR = 1 << this->pin;
        }

        mbed::PwmOut *hardware_pwm();

        // these should be private, and use getters
        DigitalInOut port;

        unsigned char pin;
        char port_number;
        struct {
            bool inverting:1;
            bool valid:1;
        };
};




#endif
