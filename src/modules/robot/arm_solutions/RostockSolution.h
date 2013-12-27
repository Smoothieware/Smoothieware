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

#define alpha_angle_checksum                CHECKSUM("alpha_angle")
#define beta_relative_angle_checksum         CHECKSUM("beta_relative_angle")
#define gamma_relative_angle_checksum        CHECKSUM("gamma_relative_angle")

class RostockSolution : public BaseSolution {
    public:
        RostockSolution(Config* passed_config);
        void millimeters_to_steps( float millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], float millimeters[] ); 

        void set_steps_per_millimeter( float steps[] );
        void get_steps_per_millimeter( float steps[] );

        float solve_arm( float millimeters[] );
        void rotate( float in[], float out[], float sin, float cos );

        Config* config;
        float alpha_steps_per_mm;
        float beta_steps_per_mm;
        float gamma_steps_per_mm;

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






#endif // ROSTOCKSOLUTION_H
