#include "Button.h"

#include "libs/Kernel.h"
#include "libs/utils.h"
#include "libs/Pin.h"
#include "libs/Hook.h"

Button::Button()
{
    this->counter = 0;
    this->value = false;
    this->up_hook = NULL;
    this->down_hook = NULL;
    this->button_pin = NULL;
    this->repeat = false;
    this->first_timer = 0;
    this->second_timer = 0;
    this->longpress_delay = 0;
}

Button *Button::pin(Pin *passed_pin)
{
    this->button_pin = passed_pin;
    return this;
}

void Button::check_signal()
{
    check_signal(this->button_pin->get() ? 1 : 0);
}

void Button::check_signal(int val)
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
                this->first_timer = 0;
                this->second_timer = 0;
                this->repeat = false;
            }
        } else {
            if ( this->down_hook != NULL ) {
                this->down_hook->call();
            }
        }
    }
    //auto repeat button presses
    if(this->longpress_delay > 0) {
        if(this->value) {
            if(this->repeat) {
                this->second_timer++;
                if(this->second_timer == 10) {
                    this->up_hook->call();
                    this->second_timer = 0;
                }
            } else {
                this->first_timer++;
                if(this->first_timer == longpress_delay) {
                    this->repeat = true;
                    this->first_timer = 0;
                }
            }
        }
    }
}

void Button::set_longpress_delay(int delay)
{
    this->longpress_delay = delay;
}

bool Button::get()
{
    return this->value;
}
