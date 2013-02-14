#ifndef ROSTOCKSOLUTION_H
#define ROSTOCKSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_steps_per_mm_checksum CHECKSUM("alpha_steps_per_mm")
#define beta_steps_per_mm_checksum  CHECKSUM("beta_steps_per_mm")
#define gamma_steps_per_mm_checksum CHECKSUM("gamma_steps_per_mm")

#define arm_length_checksum         CHECKSUM("arm_length")
#define arm_radius_checksum         CHECKSUM("arm_radius")

#define beta_angle_checksum         CHECKSUM("beta_angle")
#define gamma_angle_checksum        CHECKSUM("gamma_angle")

class RostockSolution : public BaseSolution {
    public:
        RostockSolution(Config* passed_config);
        void millimeters_to_steps( double millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], double millimeters[] ); 

        void set_steps_per_millimeter( double steps[] );
        void get_steps_per_millimeter( double steps[] );

        double solve_arm( double millimeters[] );
        void rotate_beta( double coords[], double out[] );
        void rotate_gamma( double coords[], double out[] );

        Config* config;
        double alpha_steps_per_mm;
        double beta_steps_per_mm;
        double gamma_steps_per_mm;

        double arm_length;
        double arm_radius;
        double arm_length_squared;

        double sin_beta;
        double cos_beta;
        double sin_gamma;
        double cos_gamma;
};






#endif // ROSTOCKSOLUTION_H
