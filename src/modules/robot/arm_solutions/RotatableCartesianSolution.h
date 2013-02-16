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
        void millimeters_to_steps( double millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], double millimeters[] );

        void set_steps_per_millimeter( double steps[] );
        void get_steps_per_millimeter( double steps[] );

        void rotate( double in[], double out[], double sin, double cos );

        Config* config;
        double alpha_steps_per_mm;
        double beta_steps_per_mm;
        double gamma_steps_per_mm;

        double sin_alpha;
        double cos_alpha;
};


#endif // ROTATABLECARTESIANSOLUTION_H

