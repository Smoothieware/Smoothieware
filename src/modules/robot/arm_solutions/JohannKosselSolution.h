#ifndef JOHANNKOSSELSOLUTION_H
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

class JohannKosselSolution : public BaseSolution {
    public:
        JohannKosselSolution(Config* passed_config);
        void millimeters_to_steps( float millimeters[], int steps[] );
        void steps_to_millimeters( int steps[], float millimeters[] );

        void set_steps_per_millimeter( float steps[] );
        void get_steps_per_millimeter( float steps[] );
        bool set_optional(char parameter, float value);
        bool get_optional(char parameter, float *value);

    private:
        void init();

        Config* config;
        float alpha_steps_per_mm;
        float beta_steps_per_mm;
        float gamma_steps_per_mm;

        float arm_length;
        float arm_radius;
        float arm_length_squared;

        float DELTA_TOWER1_X;
        float DELTA_TOWER1_Y;
        float DELTA_TOWER2_X;
        float DELTA_TOWER2_Y;
        float DELTA_TOWER3_X;
        float DELTA_TOWER3_Y;
};
#endif // JOHANNKOSSELSOLUTION_H
