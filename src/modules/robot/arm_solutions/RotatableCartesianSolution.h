#ifndef ROTATABLECARTESIANSOLUTION_H
#define ROTATABLECARTESIANSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_angle_checksum        CHECKSUM("alpha_angle")

class RotatableCartesianSolution : public BaseSolution {
    public:
        RotatableCartesianSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );

        void rotate( float in[], float out[], float sin, float cos );

        float sin_alpha;
        float cos_alpha;
};


#endif // ROTATABLECARTESIANSOLUTION_H

