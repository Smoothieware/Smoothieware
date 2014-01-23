#include "CartesianSolution.h"
#include <math.h>

void CartesianSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] ){
    actuator_mm[ALPHA_STEPPER] = cartesian_mm[X_AXIS];
    actuator_mm[BETA_STEPPER ] = cartesian_mm[Y_AXIS];
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void CartesianSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] ){
    cartesian_mm[ALPHA_STEPPER] = actuator_mm[X_AXIS];
    cartesian_mm[BETA_STEPPER ] = actuator_mm[Y_AXIS];
    cartesian_mm[GAMMA_STEPPER] = actuator_mm[Z_AXIS];
}
