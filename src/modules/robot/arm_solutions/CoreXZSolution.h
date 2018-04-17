#pragma once

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

class CoreXZSolution : public BaseSolution {
    public:
        CoreXZSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates & ) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

    private:
        float x_reduction;
        float z_reduction;
};
