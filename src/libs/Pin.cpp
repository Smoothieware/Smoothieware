#include "Pin.h"
#include "utils.h"

// mbed libraries for hardware pwm
#include "PwmOut.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include "pinmap.h"

Pin::Pin(PinName p){
    this->inverting = false;
    this->valid = true;
    this->pin = STM_PIN(p);
    this->port_number = STM_PORT(p);
    this->pin_name = p;
    this->pin_value = false;

    switch(STM_PORT(p)) {
        case 0: this->port = GPIOA; break;
        case 1: this->port = GPIOB; break;
        case 2: this->port = GPIOC; break;
        case 3: this->port = GPIOD; break;
        case 7: this->port = GPIOH; break;
        default: this->valid = false; break;
    }
}

GPIO_TypeDef* Pin::get_port(char port)
{
    switch (port) {
        case 'A': return GPIOA;
        case 'B': return GPIOB;
        case 'C': return GPIOC;
        case 'D': return GPIOD;
        case 'H': return GPIOH;
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
        this->port = get_port(cn[-1]);
        this->port_number = cn[-1] - 'A';
        // next char after the '.'
        cs = cn + 1;
        // if the char after the first integer is a . then we should expect a pin index next
        // grab pin index.
        this->pin = strtol(cs, &cn, 10);

        // if strtol read some numbers, cn will point to the first non-digit
        if ((cn > cs) && (pin < 16)){

            this->pin_name = (PinName) ((this->port_number << 4) | this->pin);
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
    port_number = 0xFF;
    pin = 32;
    inverting = false;
    return this;
}

extern const PinMap PinMap_PWM[];

// If available on this pin, return mbed hardware pwm class for this pin
PwmOut* Pin::hardware_pwm()
{
    int i = 0;

    if (!this->valid)
        return nullptr;

    while(PinMap_PWM[i].pin != NC) {
        if (PinMap_PWM[i].pin == this->pin_name) {
                return new PwmOut(this->pin_name);
        }
        i++;
    }
    
    return nullptr;
}

InterruptIn* Pin::interrupt_pin()
{
    if(!this->valid) return nullptr;

    // set as input
    as_input();

    return new InterruptIn(this->pin_name);
}
