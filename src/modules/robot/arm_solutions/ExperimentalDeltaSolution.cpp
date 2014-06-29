#include "ExperimentalDeltaSolution.h"

#include <fastmath.h>
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"
#include "checksumm.h"

#define arm_length_checksum         CHECKSUM("arm_length")
#define arm_radius_checksum         CHECKSUM("arm_radius")

#define alpha_angle_checksum                CHECKSUM("alpha_angle")
#define beta_relative_angle_checksum         CHECKSUM("beta_relative_angle")
#define gamma_relative_angle_checksum        CHECKSUM("gamma_relative_angle")

#define PIOVER180       0.01745329251994329576923690768489F

// NOTE this currently does not work, needs FK and settings
ExperimentalDeltaSolution::ExperimentalDeltaSolution(Config* config)
{
    float alpha_angle  = PIOVER180 * config->value(alpha_angle_checksum)->by_default(30.0f)->as_number();
    sin_alpha     = sinf(alpha_angle);
    cos_alpha     = cosf(alpha_angle);
    float beta_angle   = PIOVER180 * config->value( beta_relative_angle_checksum)->by_default(120.0f)->as_number();
    sin_beta      = sinf(beta_angle);
    cos_beta      = cosf(beta_angle);
    float gamma_angle  = PIOVER180 * config->value(gamma_relative_angle_checksum)->by_default(240.0f)->as_number();
    sin_gamma     = sinf(gamma_angle);
    cos_gamma     = cosf(gamma_angle);

    // arm_length is the length of the arm from hinge to hinge
    arm_length         = config->value(arm_length_checksum)->by_default(250.0f)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    arm_radius         = config->value(arm_radius_checksum)->by_default(124.0f)->as_number();

    arm_length_squared = powf(arm_length, 2);
}

void ExperimentalDeltaSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] ){
    float alpha_rotated[3], rotated[3];

    if( sin_alpha == 0 && cos_alpha == 1){
        alpha_rotated[X_AXIS] = cartesian_mm[X_AXIS];
        alpha_rotated[Y_AXIS] = cartesian_mm[Y_AXIS];
        alpha_rotated[Z_AXIS] = cartesian_mm[Z_AXIS];
    }else{
        rotate( cartesian_mm, alpha_rotated, sin_alpha, cos_alpha );
    }
    actuator_mm[ALPHA_STEPPER] = solve_arm( alpha_rotated );

    rotate( alpha_rotated, rotated, sin_beta, cos_beta );
    actuator_mm[BETA_STEPPER ] = solve_arm( rotated );

    rotate( alpha_rotated, rotated, sin_gamma, cos_gamma );
    actuator_mm[GAMMA_STEPPER] = solve_arm( rotated );
}

void ExperimentalDeltaSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] ){
    // unimplemented
}

float ExperimentalDeltaSolution::solve_arm( float cartesian_mm[]) {
    return sqrtf(arm_length_squared - powf(cartesian_mm[X_AXIS] - arm_radius, 2) - powf(cartesian_mm[Y_AXIS], 2)) + cartesian_mm[Z_AXIS];
}

void ExperimentalDeltaSolution::rotate(float in[], float out[], float sin, float cos ){
    out[X_AXIS] = cos * in[X_AXIS] - sin * in[Y_AXIS];
    out[Y_AXIS] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    out[Z_AXIS] = in[Z_AXIS];
}
