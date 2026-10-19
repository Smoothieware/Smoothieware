#include "BacklashCompensator.h"
#include "StepperMotor.h"

#include <math.h>

BacklashCompensator::BacklashCompensator()
{
    for (int i = 0; i < MAX_ROBOT_ACTUATORS; i++) {
        backlash_mm[i] = 0.0F;
        last_direction[i] = 0;
    }
}

void BacklashCompensator::set_backlash_mm(uint8_t motor, float mm)
{
    if (motor >= MAX_ROBOT_ACTUATORS) return;
    backlash_mm[motor] = (!isfinite(mm) || mm < 0.0F || mm > 100.0F) ? 0.0F : mm; // NaN, infinite, negative or absurd (> 100 mm) means off
}

bool BacklashCompensator::is_enabled() const
{
    if (!enabled) return false;
    for (int i = 0; i < MAX_ROBOT_ACTUATORS; i++) {
        if (backlash_mm[i] > 0.0F) return true;
    }
    return false;
}

uint32_t BacklashCompensator::slack_steps(uint8_t motor, float steps_per_mm, int32_t nominal_steps) const
{
    if (motor >= MAX_ROBOT_ACTUATORS || nominal_steps == 0 || !enabled || backlash_mm[motor] <= 0.0F) return 0;
    int8_t direction = nominal_steps > 0 ? 1 : -1;
    if (last_direction[motor] == 0 || last_direction[motor] == direction) return 0;
    return (uint32_t)lroundf(backlash_mm[motor] * steps_per_mm);
}

uint32_t BacklashCompensator::commit(uint8_t motor, float steps_per_mm, int32_t nominal_steps)
{
    if (motor >= MAX_ROBOT_ACTUATORS || nominal_steps == 0) return 0;
    uint32_t slack = slack_steps(motor, steps_per_mm, nominal_steps);
    // the direction is tracked even while backlash is off, so enabling it later (M425) starts from the right state
    last_direction[motor] = nominal_steps > 0 ? 1 : -1;
    return slack;
}

void BacklashCompensator::on_flush(StepperMotor *const *motors, uint8_t n_motors)
{
    for (uint8_t i = 0; i < n_motors && i < MAX_ROBOT_ACTUATORS; i++) {
        if (last_direction[i] == 0) continue;
        // StepperMotor::which_direction() is the direction bit: true = counter decreases
        last_direction[i] = motors[i]->which_direction() ? -1 : 1;
    }
}

void BacklashCompensator::forget_directions()
{
    for (int i = 0; i < MAX_ROBOT_ACTUATORS; i++) last_direction[i] = 0;
}
