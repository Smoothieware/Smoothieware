#include "MultiCartesianSolution.h"
#include "ActuatorCoordinates.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/Config.h"

#include <math.h>

//This solution allows driving multiple actuators for each axis. 
//Handily, it also serves as a test bed for >3 actuators.
//Number and axes for the actuators are in axis_map.
//The lowest digit is the actuator count. Each higher digit is the axis for the next actuator.
//So a conventional cartesion is 2103, and for 6 motors is 2102106.
//You should have at least one motor per axis.
//You'll also want to configure step/dir/enable/steps_per_mm/max_rate for your delta, epsilon, and zeta motors

#define axis_map_checksum CHECKSUM("multi_cartesian_axes")

MultiCartesianSolution::MultiCartesianSolution(Config* config)
{
    int axis_map =  std::max(config->value(axis_map_checksum)->by_default(102105)->as_int(), 0);
    int motor_count = std::min(k_max_actuators, axis_map % 10);
    axis_for_actuator.reserve(motor_count);

    for (int i=0; i < motor_count; i++) {
        axis_map /= 10;
        axis_for_actuator.push_back(std::min(axis_map % 10, 2));
    }

    for (int i=motor_count; i--;) {
        actuator_for_axis[axis_for_actuator[i]] = i;  //prefer low actuators
    }
}

void MultiCartesianSolution::cartesian_to_actuator( const float cartesian_mm[], ActuatorCoordinates &actuator_mm ){
    for (size_t i = 0; i < get_actuator_count(); i++) {
        actuator_mm[i] = cartesian_mm[axis_for_actuator[i]];
    }
}

void MultiCartesianSolution::actuator_to_cartesian( const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ){
    for (size_t i = 0; i < 3; i++) {
        cartesian_mm[i] = actuator_mm[actuator_for_axis[i]];
    }
}

size_t MultiCartesianSolution::get_actuator_count() const { 
    return axis_for_actuator.size(); 
}
