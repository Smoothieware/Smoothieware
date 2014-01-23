#include "HBotSolution.h"
#include <math.h>

void HBotSolution::cartesian_to_actuator( float millimeters[], float steps[] ){
    steps[ALPHA_STEPPER] = millimeters[X_AXIS] + millimeters[Y_AXIS];
    steps[BETA_STEPPER ] = millimeters[X_AXIS] - millimeters[Y_AXIS];
    steps[GAMMA_STEPPER] = millimeters[Z_AXIS];
}

void HBotSolution::actuator_to_cartesian( float steps[], float millimeters[] ){
    millimeters[ALPHA_STEPPER] = 0.5F * (steps[X_AXIS] + steps[Y_AXIS]);
    millimeters[BETA_STEPPER ] = 0.5F * (steps[X_AXIS] - steps[Y_AXIS]);
    millimeters[GAMMA_STEPPER] = steps[Z_AXIS];
}
