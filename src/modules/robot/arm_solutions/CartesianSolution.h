#ifndef CARTESIANSOLUTION_H
#define CARTESIANSOLUTION_H
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define alpha_steps_per_mm_checksum CHECKSUM("alpha_steps_per_mm")
#define beta_steps_per_mm_checksum  CHECKSUM("beta_steps_per_mm")
#define gamma_steps_per_mm_checksum CHECKSUM("gamma_steps_per_mm")

class CartesianSolution : public BaseSolution {
    public:
        CartesianSolution(Config* passed_config);
        void millimeters_to_steps( float millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], float millimeters[] );

        void set_steps_per_millimeter( float steps[] );
        void get_steps_per_millimeter( float steps[] );

        Config* config;
        float alpha_steps_per_mm;
        float beta_steps_per_mm;
        float gamma_steps_per_mm;
};






#endif
