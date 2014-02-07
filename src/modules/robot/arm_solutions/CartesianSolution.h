#ifndef CARTESIANSOLUTION_H
#define CARTESIANSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class CartesianSolution : public BaseSolution {
    public:
        CartesianSolution(){};
        CartesianSolution(Config*){};
        void cartesian_to_actuator( float millimeters[], float steps[] );
        void actuator_to_cartesian( float steps[], float millimeters[] );
        inline bool confirm_homing_set(uint8_t actuators[]) { return true; } // any set is fine
};






#endif
