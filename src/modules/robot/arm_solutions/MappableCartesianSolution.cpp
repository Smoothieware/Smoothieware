#include "MappableCartesianSolution.h"
#include "ActuatorCoordinates.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"


// see the header for module documentatation
// Example:
//
// # Arm solution configuration : MappableCartesian robot. Translates mm positions into stepper positions
// # set up for dual Y using the B motor (X=0, Y=1, Z=2, A=3, B=4, C=5)
// arm_solution                                 mappable_cartesian
// axis_slave_y                                  4

#define NO_STEPPER 255

#define axis_map_x_checksum    CHECKSUM("axis_map_x")
#define axis_map_y_checksum    CHECKSUM("axis_map_y")
#define axis_map_z_checksum    CHECKSUM("axis_map_z")
#define axis_slave_x_checksum    CHECKSUM("axis_slave_x")
#define axis_slave_y_checksum    CHECKSUM("axis_slave_y")
#define axis_slave_z_checksum    CHECKSUM("axis_slave_z")

// set up default configuration for mapper
void MappableCartesianSolution::map_default_axes()
{
    THEKERNEL->streams->printf("Map default coords\r\n");
    // the default mapping of X->alpha, Y->beta, etc.
    axis_map[X_AXIS] = ALPHA_STEPPER;
    axis_map[Y_AXIS] = BETA_STEPPER;
    axis_map[Z_AXIS] = GAMMA_STEPPER;
    axis_dual_map[X_AXIS] = NO_STEPPER;
    axis_dual_map[Y_AXIS] = NO_STEPPER;
    axis_dual_map[Z_AXIS] = NO_STEPPER;
}

MappableCartesianSolution::MappableCartesianSolution()
{
    THEKERNEL->streams->printf("Mappable cartesian default build");
    map_default_axes();
}

MappableCartesianSolution::MappableCartesianSolution(Config* config)
{
    THEKERNEL->streams->printf("Mappable cartesian config build");
    map_default_axes();
    // read optional overrides
    axis_map[X_AXIS] = config->value(axis_map_x_checksum)->by_default(axis_map[X_AXIS])->as_int();
    axis_map[Y_AXIS] = config->value(axis_map_y_checksum)->by_default(axis_map[Y_AXIS])->as_int();
    axis_map[Z_AXIS] = config->value(axis_map_z_checksum)->by_default(axis_map[Z_AXIS])->as_int();
    axis_dual_map[X_AXIS] = config->value(axis_slave_x_checksum)->by_default(axis_dual_map[X_AXIS])->as_int();
    axis_dual_map[Y_AXIS] = config->value(axis_slave_y_checksum)->by_default(axis_dual_map[Y_AXIS])->as_int();
    axis_dual_map[Z_AXIS] = config->value(axis_slave_z_checksum)->by_default(axis_dual_map[Z_AXIS])->as_int();
    // display the remapping on theconsole
    if(THEKERNEL->streams != NULL)
    {
        std::string axs[3];
        for(int i=0; i<3; i++)
        {
            if(axis_dual_map[i] == NO_STEPPER)
                axs[i] = std::to_string(axis_map[i]);
            else
            {
                axs[i] = std::to_string(axis_map[i]) + "&" + std::to_string(axis_dual_map[i]);
            }
        }
        THEKERNEL->streams->printf("Actuator mapping-> X=%s, Y=%s, Z=%s\r\n", axs[0].c_str(), axs[1].c_str(), axs[2].c_str());
        THEKERNEL->streams->printf("Maximum actuators = %d\r\n", (int)k_max_actuators);
    }

}

void MappableCartesianSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    THEKERNEL->streams->printf("Start coords %f %f %f %f %f\r\n", actuator_mm[0], actuator_mm[1], actuator_mm[2], actuator_mm[3], actuator_mm[4]);
    actuator_mm[axis_map[X_AXIS]] = cartesian_mm[X_AXIS];
    actuator_mm[axis_map[Y_AXIS]] = cartesian_mm[Y_AXIS];
    actuator_mm[axis_map[Z_AXIS]] = cartesian_mm[Z_AXIS];
    if(axis_dual_map[X_AXIS] != NO_STEPPER)
    {
        actuator_mm[axis_dual_map[X_AXIS]] = cartesian_mm[X_AXIS];
    }
    if(axis_dual_map[Y_AXIS] != NO_STEPPER)
    {
        actuator_mm[axis_dual_map[Y_AXIS]] = cartesian_mm[Y_AXIS];
    }
    if(axis_dual_map[Z_AXIS] != NO_STEPPER)
    {
        actuator_mm[axis_dual_map[Z_AXIS]] = cartesian_mm[Z_AXIS];
    }
    THEKERNEL->streams->printf("End coords %f %f %f %f %f\r\n", actuator_mm[0], actuator_mm[1], actuator_mm[2], actuator_mm[3], actuator_mm[4]);
}

void MappableCartesianSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    cartesian_mm[X_AXIS]] = actuator_mm[axis_map[X_AXIS];
    cartesian_mm[Y_AXIS] = actuator_mm[axis_map[Y_AXIS]];
    cartesian_mm[Z_AXIS] = actuator_mm[axis_map[Z_AXIS]];
}


