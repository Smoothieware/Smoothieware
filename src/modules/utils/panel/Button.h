#ifndef BUTTON_H
#define BUTTON_H

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "libs/Hook.h"


class Button
{
public:
    Button()
    {
        this->counter = 0;
        this->value = false;
        this->up_hook = NULL;
        this->down_hook = NULL;
        this->button_pin = NULL;
    }

    Button *pin(Pin *passed_pin)
    {
        this->button_pin = passed_pin;
        return this;
    }

    void check_signal()
    {
        check_signal(this->button_pin->get() ? 1 : 0);
    }

    void check_signal(int val)
    {
        bool start_value = this->value;
        if ( val ) {
            if ( this->counter < 5  ) {
                this->counter++;
            }
            if ( this->counter == 5 ) {
                this->value = true;
            }
        } else {
            if ( this->counter > 0  ) {
                this->counter--;
            }
            if ( this->counter == 0 ) {
                this->value = false;
            }
        }

        if ( start_value != this->value ) {
            if ( this->value ) {
                if ( this->up_hook != NULL ) {
                    this->up_hook->call();
                }
            } else {
                if ( this->down_hook != NULL ) {
                    this->down_hook->call();
                }
            }
        }

    }

    bool get()
    {
        return this->value;
    }


    template<typename T> Button *up_attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) )
    {
        this->up_hook = new Hook();
        this->up_hook->attach(optr, fptr);
        return this;
    }

    template<typename T> Button *down_attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) )
    {
        this->down_hook = new Hook();
        this->down_hook->attach(optr, fptr);
        return this;
    }

private:
    Hook *up_hook;
    Hook *down_hook;
    bool value;
    char counter;
    Pin *button_pin;

};





#endif
