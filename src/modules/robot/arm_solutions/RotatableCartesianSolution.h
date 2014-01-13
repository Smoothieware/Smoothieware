#ifndef ROTATABLECARTESIANSOLUTION_H
#define ROTATABLECARTESIANSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_steps_per_mm_checksum CHECKSUM("alpha_steps_per_mm")
#define beta_steps_per_mm_checksum  CHECKSUM("beta_steps_per_mm")
#define gamma_steps_per_mm_checksum CHECKSUM("gamma_steps_per_mm")

#define alpha_angle_checksum        CHECKSUM("alpha_angle")

class RotatableCartesianSolution : public BaseSolution {
    public:
        RotatableCartesianSolution(Config* passed_config);
        void millimeters_to_steps( float millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], float millimeters[] );

        void set_steps_per_millimeter( float steps[] );
        void get_steps_per_millimeter( float steps[] );

        void rotate( float in[], float out[], float sin, float cos );

        Config* config;
        float alpha_steps_per_mm;
        float beta_steps_per_mm;
        float gamma_steps_per_mm;

        float sin_alpha;
        float cos_alpha;
};


#endif // ROTATABLECARTESIANSOLUTION_H

