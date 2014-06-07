#include "Pwm.h"

#include "nuts_bolts.h"

#define PID_PWM_MAX 256

// What ?

Pwm::Pwm()
{
    _max = PID_PWM_MAX - 1;
    _pwm = -1;
    _sd_direction= false;
    _sd_accumulator= 0;
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
    if ((_pwm < 0) || _pwm >= PID_PWM_MAX) {
        return dummy;
    }
    else if (_pwm == 0) {
        Pin::set(false);
        return dummy;
    }
    else if (_pwm == PID_PWM_MAX - 1) {
        Pin::set(true);
        return dummy;
    }

    /*
     * Sigma-Delta PWM algorithm
     *
     * This Sigma-Delta implementation works by increasing _sd_accumulator by _pwm until we reach _half_ of max,
     * then decreasing by (max - target_pwm) until we hit zero
     *
     * While we're increasing, the output is 0 and while we're decreasing the output is 1
     *
     * For example, with pwm=128 and a max of 256, we'll see the following pattern:
     * ACC  ADD OUT
     *   0  128   1 // after the add, we hit 256/2 = 128 so we change direction
     * 128 -128   0 // after the add, we hit 0 so we change direction again
     *   0  128   1
     * 128 -128   0
     *  as expected
     *
     * with a pwm value of 192 (75%) we'll see this:
     *  ACC  ADD OUT
     *    0  192   0 // after the add, we are beyond max/2 so we change direction
     *  192  -64   1 // haven't reached 0 yet
     *  128  -64   1 // haven't reached 0 yet
     *   64  -64   1 // after this add we reach 0, and change direction
     *    0  192   0
     *  192  -64   1
     *  128  -64   1
     *   64  -64   1
     *    0  192   0
     * etcetera
     *
     * with a pwm value of 75 (about 29%) we'll see this pattern:
     *  ACC  ADD OUT
     *    0   75   0
     *   75   75   0
     *  150 -181   1
     *  -31   75   0
     *   44   75   0
     *  119   75   0
     *  194 -181   1
     *   13 -181   1
     * -168   75   0
     *  -93   75   0
     *  -18   75   0
     *   57   75   0
     *  132 -181   1
     *  -49   75   0
     *   26   75   0
     *  101   75   0
     *  176 -181   1
     *   -5   75   0
     *   70   75   0
     *  145 -181   1
     *  -36   75   0
     * etcetera. This pattern has 6 '1's over a total of 21 lines which is on 28.57% of the time. If we let it run longer, it would get closer to the target as time went on
     */

    // this line should never actually do anything, it's just a sanity check in case our accumulator gets corrupted somehow.
    // If we didn't check and the accumulator is corrupted, we could leave a heater on for quite a long time
    // the accumulator is kept within these limits by the normal operation of the Sigma-Delta algorithm
    _sd_accumulator = confine(_sd_accumulator, -PID_PWM_MAX, PID_PWM_MAX << 1);

    // when _sd_direction == false, our output is 0 and our accumulator is increasing by _pwm
    if (_sd_direction == false)
    {
        // increment accumulator
        _sd_accumulator += _pwm;
        // if we've reached half of max, flip our direction
        if (_sd_accumulator >= (PID_PWM_MAX >> 1))
            _sd_direction = true;
    }
    // when _sd_direction == true, our output is 1 and our accumulator is decreasing by (MAX - _pwm)
    else
    {
        // decrement accumulator
        _sd_accumulator -= (PID_PWM_MAX - _pwm);
        // if we've reached 0, flip our direction
        if (_sd_accumulator <= 0)
            _sd_direction = false;
    }
    Pin::set(_sd_direction);

    return dummy;
}
