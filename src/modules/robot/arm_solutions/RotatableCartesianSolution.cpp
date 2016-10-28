#include "RotatableCartesianSolution.h"
#include <math.h>
#include "ActuatorCoordinates.h"
#include "checksumm.h"
#include "ConfigValue.h"

// degrees * (pi / 180) = radians
#define DEG2RAD       0.01745329251994329576923690768489

RotatableCartesianSolution::RotatableCartesianSolution(Config* config)
{
    float alpha_angle  = config->value(alpha_angle_checksum)->by_default(0.0f)->as_number() * DEG2RAD;
    sin_alpha          = sinf(alpha_angle);
    cos_alpha          = cosf(alpha_angle);
}

void RotatableCartesianSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    rotate( cartesian_mm, &actuator_mm[0], sin_alpha, cos_alpha );
}

void RotatableCartesianSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    rotate( &actuator_mm[0], cartesian_mm, - sin_alpha, cos_alpha );
}

void RotatableCartesianSolution::rotate(const float in[], float out[], float sin, float cos ) const
{
    out[ALPHA_STEPPER] = cos * in[X_AXIS] - sin * in[Y_AXIS];
    out[BETA_STEPPER ] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    out[GAMMA_STEPPER] =       in[Z_AXIS];
}
