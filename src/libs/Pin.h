#ifndef PIN_H
#define PIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "libs/LPC17xx/sLPC17xx.h" // smoothed mbed.h lib

// class Pin;

// #include "libs/Kernel.h"
// #include "libs/utils.h"

#define PIN_PWM_MAX 256

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

        Pin* pull_up(void);

        Pin* pull_down(void);

        inline bool get(){
            if (this->pin >= 32) return false;
            return this->inverting ^ (( this->port->FIOPIN >> this->pin ) & 1);
        }

        inline void set(bool value)
        {
            if (this->pin >= 32) return;
            _pwm = -1;
            _set(value);
        }

        inline void _set(bool value)
        {
            if ( this->inverting ^ value )
                this->port->FIOSET = 1 << this->pin;
            else
                this->port->FIOCLR = 1 << this->pin;
        }

        inline void pwm(int value)
        {
            if (value >= PIN_PWM_MAX)
                value = (PIN_PWM_MAX - 1);
            if (value < 0)
                value = 0;
            _pwm = value;
        }

        // SIGMA-DELTA modulator
        uint32_t tick(uint32_t dummy);

        bool inverting;
        LPC_GPIO_TypeDef* port;
        char port_number;
        char pin;
        int _pwm;

        // SIGMA-DELTA pwm counters
        int _sd_accumulator;
        bool _sd_direction;
};




#endif
