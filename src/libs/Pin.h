#ifndef PIN_H
#define PIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "libs/LPC17xx/sLPC17xx.h" // smoothed mbed.h lib

class Pin {
    public:
        Pin();

        Pin* from_string(std::string value);

        inline bool connected(){
            return this->pin < 32;
        }

        inline Pin* as_output(){
            if (this->pin < 32)
                this->port->FIODIR |= 1<<this->pin;
            return this;
        }

        inline Pin* as_input(){
            if (this->pin < 32)
                this->port->FIODIR &= ~(1<<this->pin);
            return this;
        }

        Pin* as_open_drain(void);

        Pin* as_repeater(void);

        Pin* pull_up(void);

        Pin* pull_down(void);

        Pin* pull_none(void);

        inline bool get(){

            if (this->pin >= 32) return false;
            return this->inverting ^ (( this->port->FIOPIN >> this->pin ) & 1);
        }

        inline void set(bool value)
        {
            if (this->pin >= 32) return;
            if ( this->inverting ^ value )
                this->port->FIOSET = 1 << this->pin;
            else
                this->port->FIOCLR = 1 << this->pin;
        }

        LPC_GPIO_TypeDef* port;
        bool inverting;
        char port_number;
        unsigned char pin;
};




#endif
