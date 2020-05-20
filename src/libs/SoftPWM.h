#pragma once

#define POSITIVE true
#define NEGATIVE false

#include "mbed.h"

class Pin;

class SoftPWM
{
private:
    Timeout _timeout;
    Ticker _ticker;
    Pin *pulse;
    bool positive;
    float width;
    float interval;
    void TickerInterrapt();
    void end();
public:
    SoftPWM(Pin *pin, bool mode = true);
    void start();
    void write(float);
    float read();
    void pulsewidth(float);
    void pulsewidth_ms(int);
    void pulsewidth_us(int);
    void period(float);
    void period_ms(int);
    void period_us(int);
    void stop();
    operator float() { return read(); }
    SoftPWM& operator=(float duty) { write(duty); return *this; }

};
