#ifndef BUTTON_H
#define BUTTON_H

#include "FunctionPointer.h"

class Pin;

class Button
{
public:
    Button();

    Button *pin(Pin *passed_pin);

    void check_signal();
    void check_signal(int val);
	void set_longpress_delay(int delay);
    bool get();

    template<typename T>
    void up_attach( T* object, void (T::*member)(void)) {
      up_hook.attach(object, member);
    }
    template<typename T>
    void down_attach( T* object, void (T::*member)(void)) {
      down_hook.attach(object, member);
    }
    
private:
    mbed::FunctionPointer up_hook;
    mbed::FunctionPointer down_hook;
    bool value;
    char counter;
    Pin *button_pin;

	int longpress_delay;
	int first_timer;	//delay before starting to repeat
	int second_timer;	//time beetwen repeats
	bool repeat;
};





#endif
