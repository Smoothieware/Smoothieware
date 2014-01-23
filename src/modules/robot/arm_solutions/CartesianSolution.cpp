#include "CartesianSolution.h"
#include <math.h>

void CartesianSolution::cartesian_to_actuator( float millimeters[], float steps[] ){
    steps[ALPHA_STEPPER] = millimeters[X_AXIS];
    steps[BETA_STEPPER ] = millimeters[Y_AXIS];
    steps[GAMMA_STEPPER] = millimeters[Z_AXIS];
}

void CartesianSolution::actuator_to_cartesian( float steps[], float millimeters[] ){
    millimeters[ALPHA_STEPPER] = steps[X_AXIS];
    millimeters[BETA_STEPPER ] = steps[Y_AXIS];
    millimeters[GAMMA_STEPPER] = steps[Z_AXIS];
}
