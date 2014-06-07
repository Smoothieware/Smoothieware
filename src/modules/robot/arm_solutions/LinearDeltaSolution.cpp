#include "LinearDeltaSolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"
#include "Vector3.h"

#define arm_length_checksum         CHECKSUM("arm_length")
#define arm_radius_checksum         CHECKSUM("arm_radius")


#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)

LinearDeltaSolution::LinearDeltaSolution(Config* config)
{
    // arm_length is the length of the arm from hinge to hinge
    arm_length         = config->value(arm_length_checksum)->by_default(250.0f)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    arm_radius         = config->value(arm_radius_checksum)->by_default(124.0f)->as_number();

    init();
}

void LinearDeltaSolution::init() {
    arm_length_squared = SQ(arm_length);

    // Effective X/Y positions of the three vertical towers.
    float DELTA_RADIUS = arm_radius;

    float SIN_60   = 0.8660254037844386F;
    float COS_60   = 0.5F;

    DELTA_TOWER1_X = -SIN_60 * DELTA_RADIUS; // front left tower
    DELTA_TOWER1_Y = -COS_60 * DELTA_RADIUS;

    DELTA_TOWER2_X =  SIN_60 * DELTA_RADIUS; // front right tower
    DELTA_TOWER2_Y = -COS_60 * DELTA_RADIUS;

    DELTA_TOWER3_X = 0.0F; // back middle tower
    DELTA_TOWER3_Y = DELTA_RADIUS;
}

void LinearDeltaSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
    actuator_mm[ALPHA_STEPPER] = sqrtf(this->arm_length_squared
                                - SQ(DELTA_TOWER1_X - cartesian_mm[X_AXIS])
                                - SQ(DELTA_TOWER1_Y - cartesian_mm[Y_AXIS])
                                ) + cartesian_mm[Z_AXIS];
    actuator_mm[BETA_STEPPER ] = sqrtf(this->arm_length_squared
                                - SQ(DELTA_TOWER2_X - cartesian_mm[X_AXIS])
                                - SQ(DELTA_TOWER2_Y - cartesian_mm[Y_AXIS])
                                ) + cartesian_mm[Z_AXIS];
    actuator_mm[GAMMA_STEPPER] = sqrtf(this->arm_length_squared
                                - SQ(DELTA_TOWER3_X - cartesian_mm[X_AXIS])
                                - SQ(DELTA_TOWER3_Y - cartesian_mm[Y_AXIS])
                                ) + cartesian_mm[Z_AXIS];
}

void LinearDeltaSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] )
{
    // from http://en.wikipedia.org/wiki/Circumscribed_circle#Barycentric_coordinates_from_cross-_and_dot-products
    // based on https://github.com/ambrop72/aprinter/blob/2de69a/aprinter/printer/DeltaTransform.h#L81
    Vector3 tower1( DELTA_TOWER1_X, DELTA_TOWER1_Y, actuator_mm[0] );
    Vector3 tower2( DELTA_TOWER2_X, DELTA_TOWER2_Y, actuator_mm[1] );
    Vector3 tower3( DELTA_TOWER3_X, DELTA_TOWER3_Y, actuator_mm[2] );

    Vector3 s12 = tower1.sub(tower2);
    Vector3 s23 = tower2.sub(tower3);
    Vector3 s13 = tower1.sub(tower3);

    Vector3 normal = s12.cross(s23);

    float magsq_s12 = s12.magsq();
    float magsq_s23 = s23.magsq();
    float magsq_s13 = s13.magsq();

    float inv_nmag_sq = 1.0F / normal.magsq();
    float q = 0.5F * inv_nmag_sq;

    float a = q * magsq_s23 * s12.dot(s13);
    float b = q * magsq_s13 * s12.dot(s23) * -1.0F; // negate because we use s12 instead of s21
    float c = q * magsq_s12 * s13.dot(s23);

    Vector3 circumcenter( DELTA_TOWER1_X * a + DELTA_TOWER2_X * b + DELTA_TOWER3_X * c,
                          DELTA_TOWER1_Y * a + DELTA_TOWER2_Y * b + DELTA_TOWER3_Y * c,
                          actuator_mm[0] * a + actuator_mm[1] * b + actuator_mm[2] * c );

    float r_sq = 0.5F * q * magsq_s12 * magsq_s23 * magsq_s13;
    float dist = sqrtf(inv_nmag_sq * (arm_length_squared - r_sq));

    Vector3 cartesian = circumcenter.sub(normal.mul(dist));

    cartesian_mm[0] = ROUND(cartesian[0], 4);
    cartesian_mm[1] = ROUND(cartesian[1], 4);
    cartesian_mm[2] = ROUND(cartesian[2], 4);
}

bool LinearDeltaSolution::set_optional(const arm_options_t& options) {

    arm_options_t::const_iterator i;

    i= options.find('L');
    if(i != options.end()) {
        arm_length= i->second;

    }
    i= options.find('R');
    if(i != options.end()) {
        arm_radius= i->second;
    }
    init();
    return true;
}

bool LinearDeltaSolution::get_optional(arm_options_t& options) {
    options['L']= this->arm_length;
    options['R']= this->arm_radius;
    return true;
};
