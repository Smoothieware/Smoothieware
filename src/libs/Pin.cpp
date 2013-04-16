#include "Pin.h"

#include "utils.h"

Pin::Pin(){}

// Make a new pin object from a string
// TODO : Comment this more, how does it work ?
// TODO : Make this able to configure pull-up, pull-down, and open-drain
Pin* Pin::from_string(std::string value){
    LPC_GPIO_TypeDef* gpios[5] ={LPC_GPIO0,LPC_GPIO1,LPC_GPIO2,LPC_GPIO3,LPC_GPIO4};

    // cs is the current position in the string
    const char* cs = value.c_str();
    // cn is the position of the next char after the number we just read
    char* cn = NULL;

    // grab first integer as port. pointer to first non-digit goes in cn
    this->port_number = strtol(cs, &cn, 10);
    // if cn > cs then strtol read at least one digit
    if ((cn > cs) && (port_number <= 4)){
        // translate port index into something useful
        this->port = gpios[(unsigned int) this->port_number];
        // if the char after the first integer is a . then we should expect a pin index next
        if (*cn == '.'){
            // move pointer to first digit (hopefully) of pin index
            cs = ++cn;
            // grab pin index.
            this->pin = strtol(cs, &cn, 10);
            // if strtol read some numbers, cn will point to the first non-digit
            if ((cn > cs) & (pin < 32)){
                // skip any whitespace following the pin index
                while (is_whitespace(*cn)) cn++;
                // if we have an exclamation, we invert the pin
                this->inverting = (*cn == '!');

                this->port->FIOMASK &= ~(1 << this->pin);

                return this;
            }
        }
    }

    // from_string failed. TODO: some sort of error
    port_number = 0;
    port = gpios[0];
    pin = 255;
    inverting = false;
    return this;
}

// Configure this pin as OD
Pin* Pin::as_open_drain(){
    if (this->pin >= 32) return this;
    if( this->port_number == 0 ){ LPC_PINCON->PINMODE_OD0 |= (1<<this->pin); }
    if( this->port_number == 1 ){ LPC_PINCON->PINMODE_OD1 |= (1<<this->pin); }
    if( this->port_number == 2 ){ LPC_PINCON->PINMODE_OD2 |= (1<<this->pin); }
    if( this->port_number == 3 ){ LPC_PINCON->PINMODE_OD3 |= (1<<this->pin); }
    if( this->port_number == 4 ){ LPC_PINCON->PINMODE_OD4 |= (1<<this->pin); }
    return this;
}

// Configure this pin as a pullup
Pin* Pin::pull_up(){
    if (this->pin >= 32) return this;
    // Set the two bits for this pin as 00
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 &= ~(3<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 &= ~(3<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 &= ~(3<<((this->pin-16)*2)); }
    return this;
}

// Configure this pin as a pulldown
Pin* Pin::pull_down(){
    if (this->pin >= 32) return this;
    // Set the two bits for this pin as 11
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 |= (3<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 |= (3<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 |= (3<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 |= (3<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 |= (3<<((this->pin-16)*2)); }
    return this;
}
