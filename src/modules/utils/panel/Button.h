#ifndef BUTTON_H
#define BUTTON_H

#include <functional>

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

    void up_attach( std::function<void(void)> f);
    void down_attach( std::function<void(void)> f);
    
private:
    std::function<void(void)> up_hook;
    std::function<void(void)> down_hook;
    bool value;
    char counter;
    Pin *button_pin;

	int longpress_delay;
	int first_timer;	//delay before starting to repeat
	int second_timer;	//time beetwen repeats
	bool repeat;
};





#endif
