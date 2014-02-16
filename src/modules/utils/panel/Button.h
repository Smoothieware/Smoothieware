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
		this->repeat=false;
		this->first_timer=0;
		this->second_timer=0;
		this->longpress_delay=0;
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
					this->first_timer=0;
					this->second_timer=0;
					this->repeat=false;
                }
            } else {
                if ( this->down_hook != NULL ) {
                    this->down_hook->call();
                }
            }
        }
	//auto repeat button presses
		if(this->longpress_delay>0){
			if(this->value){
				 if(this->repeat){
					this->second_timer++;
					if(this->second_timer==10){
						this->up_hook->call();
						this->second_timer=0;
					}
				 }
				 else{
					this->first_timer++;
					if(this->first_timer==longpress_delay){
						this->repeat=true;
						this->first_timer=0;
					}
				 }
			}
		}
    }
	
	void set_longpress_delay(int delay){
		this->longpress_delay=delay;
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
	
	int longpress_delay;
	int first_timer;	//delay before starting to repeat
	int second_timer;	//time beetwen repeats
	bool repeat;
};





#endif
