#include "CoreXZSolution.h"
#include "ActuatorCoordinates.h"
#include "ConfigValue.h"
#include "checksumm.h"

#define x_reduction_checksum         CHECKSUM("x_reduction")
#define z_reduction_checksum         CHECKSUM("z_reduction")

CoreXZSolution::CoreXZSolution(Config* config)
{
    x_reduction = config->value(x_reduction_checksum)->by_default(1.0f)->as_number();
    z_reduction = config->value(z_reduction_checksum)->by_default(3.0f)->as_number();
}

void CoreXZSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const {
    actuator_mm[ALPHA_STEPPER] = (this->x_reduction * cartesian_mm[X_AXIS]) + (this->z_reduction * cartesian_mm[Z_AXIS]);
    actuator_mm[BETA_STEPPER ] = (this->x_reduction * cartesian_mm[X_AXIS]) - (this->z_reduction * cartesian_mm[Z_AXIS]);
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Y_AXIS];
}

void CoreXZSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const {
    cartesian_mm[X_AXIS] = (0.5F/this->x_reduction) * (actuator_mm[ALPHA_STEPPER] + actuator_mm[BETA_STEPPER]);
    cartesian_mm[Z_AXIS] = (0.5F/this->z_reduction) * (actuator_mm[ALPHA_STEPPER] - actuator_mm[BETA_STEPPER]);
    cartesian_mm[Y_AXIS] = actuator_mm[GAMMA_STEPPER];
}
