#pragma once

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_angle_checksum        CHECKSUM("alpha_angle")

class RotatableCartesianSolution : public BaseSolution {
    public:
        RotatableCartesianSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

    private:
        void rotate(const float in[], float out[], float sin, float cos) const;

        float sin_alpha;
        float cos_alpha;
};
