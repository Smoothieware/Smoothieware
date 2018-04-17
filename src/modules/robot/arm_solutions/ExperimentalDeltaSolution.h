#pragma once

#include "BaseSolution.h"

class Config;

class ExperimentalDeltaSolution : public BaseSolution {
    public:
        ExperimentalDeltaSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

    private:
        float solve_arm( float millimeters[] ) const ;
        void rotate(const float in[], float out[], float sin, float cos ) const ;

        float arm_length;
        float arm_radius;
        float arm_length_squared;

        float sin_alpha;
        float cos_alpha;
        float sin_beta;
        float cos_beta;
        float sin_gamma;
        float cos_gamma;
};
