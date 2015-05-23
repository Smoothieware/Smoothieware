#ifndef PIN_H
#define PIN_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "mbed.h"

#include "PinNames.h"

class Pin {
    public:
        Pin(PinName p = NC);

        Pin* from_string(std::string value);

        inline bool connected(){
            return this->valid;
        }

        inline bool equals(const Pin& other) const {
            return (this->pin == other.pin) && (this->port == other.port);
            return true;
        }

        Pin& operator= (int value) {
            set(value == 0 ? false : true);
            return *this;
        }

        Pin& operator! () {
            set(!this->pin_value);
            return *this;
        }

        inline Pin* as_output(){
            if (this->valid) {
                uint32_t moder = this->port->MODER;
                moder &= ~(3 << (this->pin * 2));
                moder |= (1 << (this->pin * 2));
                this->port->MODER = moder;
                /* Maximum output speed */
                this->port->OSPEEDR |= (1 << (this->pin * 2));
            }
            return this;
        }

        inline Pin* as_input(){
            if (this->valid) {
                this->port->MODER &= ~(3 << (this->pin * 2));
            }
            return this;
        }

        // Configure this pin as no pullup or pulldown
        inline Pin* pull_none(){
        
            if (this->valid) {
                // No pull up/down RMW
                uint32_t pupdr = this->port->PUPDR;
                pupdr &= ~(3 << (this->pin * 2));
                this->port->PUPDR = pupdr;
            }
            return this;
        }
        // Configure this pin as a pullup
        inline Pin* pull_up(){
        
            if (this->valid) {
                // pull up RMW
                uint32_t pupdr = this->port->PUPDR;
                pupdr &= ~(3 << (this->pin * 2));
                pupdr |= (1 << (this->pin * 2));
                this->port->PUPDR = pupdr;
            }
	    return this;
        }

        // Configure this pin as a pulldown
        inline Pin* pull_down(){
        
            if (this->valid) {
                // pull up RMW
                uint32_t pupdr = this->port->PUPDR;
                pupdr &= ~(3 << (this->pin * 2));
                pupdr |= (2 << (this->pin * 2));
                this->port->PUPDR = pupdr;
            }
	    return this;
        }
        
        // Configure this pin as OD
        inline Pin* as_open_drain(){
            if (this->valid) {
                // Open drain
                this->port->OTYPER |= (1 << this->pin);
            }
            pull_none();
            return this;
        }

        inline bool get(){
            if (!this->valid) return false;
            return this->inverting ^ (( this->port->IDR >> this->pin ) & 1);
        }

        inline void set(bool value)
        {
            this->pin_value = value;
            if (!this->valid) return;
            if (this->inverting ^ value)
                this->port->BSRRL = 1 << this->pin;
            else
                this->port->BSRRH = 1 << this->pin;
        }

        PwmOut *hardware_pwm();

        InterruptIn *interrupt_pin();

        GPIO_TypeDef* get_port(char port);

        // these should be private, and use getters
        GPIO_TypeDef *port;

        unsigned char pin;
        PinName pin_name;
        unsigned char port_number;
        bool pin_value;
        struct {
            bool inverting:1;
            bool valid:1;
        };
};




#endif
