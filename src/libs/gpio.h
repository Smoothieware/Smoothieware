#ifndef _GPIO_HPP
#define	_GPIO_HPP

#include <stdint.h>

#define	GPIO_DIR_INPUT 0
#define	GPIO_DIR_OUTPUT 1

#include <PinNames.h>

class GPIO {
public:
	uint8_t port;
	uint8_t pin;
    GPIO(PinName);
	GPIO(uint8_t port, uint8_t pin);
	GPIO(uint8_t port, uint8_t pin, uint8_t direction);
	// 		~GPIO();
	void setup();
	void set_direction(uint8_t direction);
	void output();
	void input();
	void write(uint8_t value);
	void set();
	void clear();
	uint8_t get();

    int operator=(int);
};

#endif /* _GPIO_HPP */
