#include "HBotSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>

void HBotSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS] + cartesian_mm[Y_AXIS];
    actuator_mm[BETA_STEPPER ] = cartesian_mm[X_AXIS] - cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void HBotSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    cartesian_mm[X_AXIS] = 0.5F * (actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER]);
    cartesian_mm[Y_AXIS] = 0.5F * (actuator_mm[ALPHA_STEPPER] - actuator_mm[BETA_STEPPER]);
    cartesian_mm[Z_AXIS] = actuator_mm[GAMMA_STEPPER];
}
