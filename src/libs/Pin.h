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
        
        inline bool supports_interrupt(){
            return this->pin < 32 && (this->port_number == 0 || this->port_number == 2);
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
        
        Pin* rising_interrupt();
        Pin* falling_interrupt();
        
        inline Pin* leading_interrupt(){
            return this->inverting ? this->falling_interrupt() : this->rising_interrupt();
        }

        inline Pin* trailing_interrupt(){
            return this->inverting ? this->rising_interrupt() : this->falling_interrupt();
        }

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
        
        bool rising_edge_seen();
        
        bool falling_edge_seen();
        
        inline bool leading_edge_seen(){
            return this->inverting ? this->falling_edge_seen() : this->rising_edge_seen();
        }
        
        inline bool trailing_edge_seen(){
            return this->inverting ? this->rising_edge_seen() : this->falling_edge_seen();
        }
        
        void clear_interrupt();

        LPC_GPIO_TypeDef* port;
        bool inverting;
        char port_number;
        unsigned char pin;
};




#endif
