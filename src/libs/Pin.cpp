#include "Pin.h"
#include "utils.h"

// mbed libraries for hardware pwm
#include "PwmOut.h"
#include "PinNames.h"

Pin::Pin(){
    this->inverting = false;
    this->valid = false;
    this->pin = 32;
    this->port = nullptr;
}

GPIO_TypeDef* Pin::get_port(char port)
{
    switch (port) {
        case 'A':
                return GPIOA;
        case 'B':
                return GPIOB;
        case 'C':
                return GPIOC;
        case 'H':
                return GPIOH;
        default:
                return nullptr;
    }
}

// Make a new pin object from a string
Pin* Pin::from_string(std::string value){

    // cs is the current position in the string
    const char* cs = value.c_str();
    // cn is the position of the next char after the number we just read
    char* cn = NULL;
    valid = true;
    cn = strchr(cs, '.');
    // if cn > cs then strtol read at least one digit
    if (cn > cs){
        // translate port index into something useful
        this->port = get_port(cn[0]);
        // next char after the '.'
        cn++;
        // if the char after the first integer is a . then we should expect a pin index next
        // grab pin index.
        this->pin = strtol(cs, &cn, 10);

        // if strtol read some numbers, cn will point to the first non-digit
        if ((cn > cs) && (pin < 16)){

            // now check for modifiers:-
            // ! = invert pin
            // o = set pin to open drain
            // ^ = set pin to pull up
            // v = set pin to pull down
            // - = set pin to no pull up or down
            // @ = set pin to repeater mode
            for (;*cn;cn++) {
                switch(*cn) {
                    case '!':
                        this->inverting = true;
                        break;
                    case 'o':
                        as_open_drain();
                        break;
                    case '^':
                        pull_up();
                        break;
                    case 'v':
                        pull_down();
                        break;
                    case '-':
                        pull_none();
                        break;
                    case '@':
                        as_repeater();
                        break;
                    default:
                        // skip any whitespace following the pin index
                        if (!is_whitespace(*cn))
                            return this;
                }
            }
            return this;
        }
    }

    // from_string failed. TODO: some sort of error
    valid= false;
    port = nullptr;
    pin = 32;
    inverting = false;
    return this;
}



// Configure this pin as a repeater
Pin* Pin::as_repeater(){
    /* FIXME STM32 
    if (!this->valid) return this;
    // Set the two bits for this pin as 01
    if( this->port_number == 0 && this->pin < 16  ){ LPC_PINCON->PINMODE0 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE0 &= ~(2<<( this->pin    *2)); }
    if( this->port_number == 0 && this->pin >= 16 ){ LPC_PINCON->PINMODE1 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE1 &= ~(2<<((this->pin-16)*2)); }
    if( this->port_number == 1 && this->pin < 16  ){ LPC_PINCON->PINMODE2 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE2 &= ~(2<<( this->pin    *2)); }
    if( this->port_number == 1 && this->pin >= 16 ){ LPC_PINCON->PINMODE3 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE3 &= ~(2<<((this->pin-16)*2)); }
    if( this->port_number == 2 && this->pin < 16  ){ LPC_PINCON->PINMODE4 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE4 &= ~(2<<( this->pin    *2)); }
    if( this->port_number == 3 && this->pin >= 16 ){ LPC_PINCON->PINMODE7 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE7 &= ~(2<<((this->pin-16)*2)); }
    if( this->port_number == 4 && this->pin >= 16 ){ LPC_PINCON->PINMODE9 |= (1<<( this->pin*2)); LPC_PINCON->PINMODE9 &= ~(2<<((this->pin-16)*2)); }
    * */
    return this;
}



// If available on this pin, return mbed hardware pwm class for this pin
PwmOut* Pin::hardware_pwm()
{
    /* FIXME STM32 
    if (port_number == 1)
    {
        if (pin == 18) { return new PwmOut(P1_18); }
        if (pin == 20) { return new PwmOut(P1_20); }
        if (pin == 21) { return new PwmOut(P1_21); }
        if (pin == 23) { return new PwmOut(P1_23); }
        if (pin == 24) { return new PwmOut(P1_24); }
        if (pin == 26) { return new PwmOut(P1_26); }
    }
    else if (port_number == 2)
    {
        if (pin == 0) { return new PwmOut(P2_0); }
        if (pin == 1) { return new PwmOut(P2_1); }
        if (pin == 2) { return new PwmOut(P2_2); }
        if (pin == 3) { return new PwmOut(P2_3); }
        if (pin == 4) { return new PwmOut(P2_4); }
        if (pin == 5) { return new PwmOut(P2_5); }
    }
    else if (port_number == 3)
    {
        if (pin == 25) { return new PwmOut(P3_25); }
        if (pin == 26) { return new PwmOut(P3_26); }
    }
    * */
    return nullptr;
}
