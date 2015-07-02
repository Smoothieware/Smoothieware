#ifndef MORGANSCARASOLUTION_H
#define MORGANSCARASOLUTION_H
//#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class MorganSCARASolution : public BaseSolution {
    public:
        MorganSCARASolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );

        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);

    private:
        void init();
        float to_degrees(float radians);

        float arm1_length;
        float arm2_length;
        float morgan_offset_x;
        float morgan_offset_y;
        float morgan_scaling_x;
        float morgan_scaling_y;
        float morgan_undefined_min;
        float morgan_undefined_max;
        float slow_rate;
};

#endif // MORGANSCARASOLUTION_H
