#ifndef COREXZSOLUTION_H
#define COREXZSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class CoreXZSolution : public BaseSolution {
    public:
        CoreXZSolution(Config*);
        void cartesian_to_actuator(const float[], float[] );
        void actuator_to_cartesian(const float[], float[] );
};

#endif // COREXZSOLUTION_H
