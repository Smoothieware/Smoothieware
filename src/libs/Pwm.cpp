#include "Pwm.h"

#include "nuts_bolts.h"

#define PID_PWM_MAX 256

Pwm::Pwm()
{
    _max = PID_PWM_MAX - 1;
    _pwm = -1;
}

void Pwm::pwm(int new_pwm)
{
    _pwm = confine(new_pwm, 0, _max);
}

Pwm* Pwm::max_pwm(int new_max)
{
    _max = confine(new_max, 0, PID_PWM_MAX - 1);
    _pwm = confine(   _pwm, 0, _max);
    return this;
}

int Pwm::max_pwm()
{
    return _max;
}

void Pwm::set(bool value)
{
    _pwm = -1;
    Pin::set(value);
}

uint32_t Pwm::on_tick(uint32_t dummy)
{
    if ((_pwm < 0) || (_pwm >= PID_PWM_MAX))
        return dummy;

    _sd_accumulator = confine(_sd_accumulator, -PID_PWM_MAX, PID_PWM_MAX << 1);

    if (_sd_direction == false)
    {
        _sd_accumulator += _pwm;
        if (_sd_accumulator >= (PID_PWM_MAX >> 1))
            _sd_direction = true;
    }
    else
    {
        _sd_accumulator -= (PID_PWM_MAX - _pwm);
        if (_sd_accumulator <= 0)
            _sd_direction = false;
    }
    Pin::set(_sd_direction);

    return dummy;
}
