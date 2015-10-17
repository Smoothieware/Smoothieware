#include "MultiCartesianSolution.h"
#include "ActuatorCoordinates.h"
#include <math.h>

//This solution allows driving dual actuators for each axis. 
//Handily, it also serves as a test bed for >3 actuators.

void MultiCartesianSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm ){
    for (size_t i = 0; i < 6; i++)
        actuator_mm[i] = cartesian_mm[i%3];
}

void MultiCartesianSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ){
    for (size_t i = 0; i < 3; i++)
        cartesian_mm[i] = actuator_mm[i];
}

size_t MultiCartesianSolution::get_actuator_count() const { 
    return 6; 
}