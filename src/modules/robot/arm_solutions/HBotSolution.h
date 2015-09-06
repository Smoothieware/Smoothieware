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
        void cartesian_to_actuator(const float[], float[] );
        void actuator_to_cartesian(const float[], float[] );
};






#endif // HBOTSOLUTION_H

