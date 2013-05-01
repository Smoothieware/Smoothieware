#include "gpio.h"

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

GPIO::GPIO(PinName pin) {
    this->port = (pin >> 5) & 7;
    this->pin = pin & 0x1F;

    setup();
}

GPIO::GPIO(uint8_t port, uint8_t pin) {
	GPIO::port = port;
	GPIO::pin = pin;

	setup();
}

GPIO::GPIO(uint8_t port, uint8_t pin, uint8_t direction) {
	GPIO::port = port;
	GPIO::pin = pin;

	setup();

	set_direction(direction);
}
// GPIO::~GPIO() {}

void GPIO::setup() {
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Portnum = GPIO::port;
	PinCfg.Pinnum = GPIO::pin;
	PINSEL_ConfigPin(&PinCfg);
}

void GPIO::set_direction(uint8_t direction) {
	FIO_SetDir(port, 1UL << pin, direction);
}

void GPIO::output() {
	set_direction(1);
}

void GPIO::input() {
	set_direction(0);
}

void GPIO::write(uint8_t value) {
	output();
	if (value)
		set();
	else
		clear();
}

void GPIO::set() {
	FIO_SetValue(port, 1UL << pin);
}

void GPIO::clear() {
	FIO_ClearValue(port, 1UL << pin);
}

uint8_t GPIO::get() {
	return (FIO_ReadValue(port) & (1UL << pin))?255:0;
}

int GPIO::operator=(int value) {
    if (value)
        set();
    else
        clear();
    return value;
}
