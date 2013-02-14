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

#define rostock_arm_length_checksum         CHECKSUM("arm_length")
#define rostock_arm_radius_checksum         CHECKSUM("arm_radius")

class RostockSolution : public BaseSolution {
    public:
        RostockSolution(Config* passed_config);
        void millimeters_to_steps( double millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], double millimeters[] ); 

        void set_steps_per_millimeter( double steps[] );
        void get_steps_per_millimeter( double steps[] );

        double solve_arm( double millimeters[] );
        void rotate_120( double coords[], double out[] );
        void rotate_240( double coords[], double out[] );

        Config* config;
        double alpha_steps_per_mm;
        double beta_steps_per_mm;
        double gamma_steps_per_mm;

        double arm_length;
        double arm_radius;
        double arm_length_squared;
};






#endif // ROSTOCKSOLUTION_H
