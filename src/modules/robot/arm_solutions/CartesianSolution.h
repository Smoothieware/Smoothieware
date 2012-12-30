#ifndef CARTESIANSOLUTION_H
#define CARTESIANSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_steps_per_mm_checksum 46458
#define beta_steps_per_mm_checksum  13840
#define gamma_steps_per_mm_checksum 33143

class CartesianSolution : public BaseSolution {
    public:
        CartesianSolution(Config* passed_config);
        void millimeters_to_steps( double millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], double millimeters[] );

        void set_steps_per_millimeter( double steps[] );
        void get_steps_per_millimeter( double steps[] );

        Config* config;
        double alpha_steps_per_mm;
        double beta_steps_per_mm;
        double gamma_steps_per_mm;
};






#endif
