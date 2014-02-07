#ifndef HBOTSOLUTION_H
#define HBOTSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class HBotSolution : public BaseSolution {
    public:
        HBotSolution();
        HBotSolution(Config*){};
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );
        bool confirm_homing_set(uint8_t actuators[]);
};






#endif // HBOTSOLUTION_H

