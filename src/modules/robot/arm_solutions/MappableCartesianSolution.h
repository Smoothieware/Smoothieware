#pragma once

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"


// This supports standard X,Y,Z cartesian coordinates but allows
// Each axis to map to any desired actuator (whoopee) and 
// More importantly allows for slave motors so you can run dual X and dual Y axes
// The slave is sent commands that should be in synch with the master
// So for microstepping the slave physical motor and driver should be the same as the master I think
// Position is read from the master
// use axis_map_x and axis_slave_x for the two motors. 
// Default slave is none. Default master is X=0,Y=1,Z=2.
// In keeping with the board labels, the argument is X, Y, Z, A, B, C

class MappableCartesianSolution : public BaseSolution {
    public:
        MappableCartesianSolution();
        MappableCartesianSolution(Config*);
        void cartesian_to_actuator( const float millimeters[], ActuatorCoordinates &steps ) const override;
        void actuator_to_cartesian( const ActuatorCoordinates &steps, float millimeters[] ) const override;
    private:
        void map_default_axes(void);
        uint8_t axis_map[3];            // X, Y, Z axes
        uint8_t axis_dual_map[3];       // possible dual controllers
};
