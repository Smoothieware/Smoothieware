#include "SoftPWM.h"
#include "Pin.h"

SoftPWM::SoftPWM(Pin *_outpin, bool _positive) : pulse(_outpin)    //constructa
{
    pulse->set(!_positive);
    positive = _positive;
    interval = 0.02F;
    width = 0;
    start();
}

float SoftPWM::read()
{
    if ( width <= 0 ) return 0;
    if ( width > 1 )  return 1;
    return width / interval;
}

void SoftPWM::write(float duty)
{
    width = interval * duty;
    if ( duty <= 0 ) width =  0;
    if ( duty > 1 )  width =  interval;
}

void SoftPWM::start()
{
    _ticker.attach(this, &SoftPWM::TickerInterrapt, interval);
}

void SoftPWM::stop()
{
    _ticker.detach();
    pulse->set(!positive);
    //wait(width);
}

void SoftPWM::period(float _period)
{
    interval = _period;
    start();
}

void SoftPWM::period_ms(int _period)
{
    period((float)_period / 1000);
    start();
}

void SoftPWM::period_us(int _period)
{
    period((float)_period / 1000000);
    start();
}

void SoftPWM::pulsewidth(float _width)
{
    width = _width;
    if ( width < 0.0F ) width = 0.0F;
}

void SoftPWM::pulsewidth_ms(int _width)
{
    pulsewidth((float)_width / 1000);
}

void SoftPWM::pulsewidth_us(int _width)
{
    pulsewidth((float)_width / 1000000);
}

void SoftPWM::TickerInterrapt()
{
    if ( width <= 0 ) return;
    _timeout.attach(this, &SoftPWM::end, width);
    pulse->set(positive);
}

void SoftPWM::end()
{
    pulse->set(!positive);
}


