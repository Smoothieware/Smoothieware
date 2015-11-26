#include "GUSDeltaSolution.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"

#include <fastmath.h>
#include "Vector3.h"

#define base_length_checksum             CHECKSUM("base_length")
#define vertical_offset_checksum         CHECKSUM("vertical_offset")


#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
#define PIOVER180   0.01745329251994329576923690768489F
#define COS60       0.5F
#define SIN60       0.866025403784439F
#define SIN60_ON_3  0.288675134594813F
#define COTAN60     0.577350269189626F

GUSDeltaSolution::GUSDeltaSolution(Config* config)
{
    // base_length is the length of the arm from hinge to hinge
    base_length = config->value(base_length_checksum)->by_default(250.0f)->as_number();
	
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    vertical_offset = config->value(vertical_offset_checksum)->by_default(79.0f)->as_number();
}

void GUSDeltaSolution::cartesian_to_actuator(const float cartesian_mm[], float actuator_mm[] )
{
	float z_square, y_square_ab, y_square_c;
	z_square                   =  SQ(cartesian_mm[Z_AXIS] + vertical_offset);
	y_square_ab                =  SQ(cartesian_mm[Y_AXIS]       +     SIN60_ON_3 * base_length);
	y_square_c                 =  SQ(cartesian_mm[Y_AXIS]       - 2 * SIN60_ON_3 * base_length);
    actuator_mm[ALPHA_STEPPER] =  sqrtf(SQ(cartesian_mm[X_AXIS] + COS60 * base_length) + y_square_ab + z_square);
	actuator_mm[BETA_STEPPER]  =  sqrtf(SQ(cartesian_mm[X_AXIS] - COS60 * base_length) + y_square_ab + z_square);
	actuator_mm[GAMMA_STEPPER] =  sqrtf(SQ(cartesian_mm[X_AXIS])                     + y_square_c  + z_square);
}

void GUSDeltaSolution::actuator_to_cartesian(const float actuator_mm[], float cartesian_mm[] )
{        
float xxx, yyy, zzz;
xxx                  =      (SQ(actuator_mm[ALPHA_STEPPER]) - SQ(actuator_mm[BETA_STEPPER])  + SQ(base_length)) /(2*base_length); 
yyy                  =      (SQ(actuator_mm[ALPHA_STEPPER]) - SQ(actuator_mm[GAMMA_STEPPER]) + SQ(base_length))/ (2*SIN60*base_length) - (COTAN60 * xxx); 
zzz                  = sqrtf(SQ(actuator_mm[ALPHA_STEPPER]) - SQ(xxx) - SQ(yyy)) ; 
cartesian_mm[X_AXIS] = xxx  - COS60 * base_length; 
cartesian_mm[Y_AXIS] = yyy  - SIN60_ON_3 * base_length;
cartesian_mm[Z_AXIS] = zzz  - vertical_offset;
}
