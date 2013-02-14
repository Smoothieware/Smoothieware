#include "RostockSolution.h"
#include <math.h>

RostockSolution::RostockSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();

    // arm_length is the length of the arm from hinge to hinge
    this->arm_length         = this->config->value(rostock_arm_length_checksum)->by_default(430.0)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    this->arm_radius         = this->config->value(rostock_arm_radius_checksum)->by_default(220.675)->as_number();

    this->arm_length_squared = pow(this->arm_length, 2);
}

void RostockSolution::millimeters_to_steps( double millimeters[], int steps[] ){
    double rotated[3];
    steps[ALPHA_STEPPER] = lround( solve_arm( millimeters ) * this->alpha_steps_per_mm );

    rotate_120(millimeters, rotated);
    steps[BETA_STEPPER ] = lround( solve_arm( rotated ) * this->beta_steps_per_mm );

    rotate_240(millimeters, rotated);
    steps[GAMMA_STEPPER] = lround( solve_arm( rotated ) * this->gamma_steps_per_mm );
}

void RostockSolution::steps_to_millimeters( int steps[], double millimeters[] ){} 

double RostockSolution::solve_arm( double millimeters[]) {
    double arm_xlift = sqrt( arm_length_squared - pow( this->arm_radius - millimeters[X_AXIS], 2 ) );
    double arm_ylift = sqrt( pow( arm_xlift, 2) - pow( - millimeters[Y_AXIS], 2 ) ) + arm_xlift;
    return arm_xlift - arm_ylift + millimeters[Z_AXIS];
}

#define SIN60 0.8660254037844386
#define COS60 0.5

#define SIN120 SIN60
#define COS120 -COS60
 
#define SIN240 -SIN60
#define COS240 -COS60
 
void RostockSolution::rotate_120( double in[], double out[] ){
    out[X_AXIS] = COS120 * in[X_AXIS] - SIN120 * in[Y_AXIS];
    out[Y_AXIS] = SIN120 * in[X_AXIS] + COS120 * in[Y_AXIS];
    out[Z_AXIS] = in[Z_AXIS];
}

void RostockSolution::rotate_240( double in[], double out[] ){
    out[X_AXIS] = COS240 * in[X_AXIS] - SIN240 * in[Y_AXIS];
    out[Y_AXIS] = SIN240 * in[X_AXIS] + COS240 * in[Y_AXIS];
    out[Z_AXIS] = in[Z_AXIS];
}

void RostockSolution::set_steps_per_millimeter( double steps[] )
{
    this->alpha_steps_per_mm = steps[0];
    this->beta_steps_per_mm  = steps[1];
    this->gamma_steps_per_mm = steps[2];
}

void RostockSolution::get_steps_per_millimeter( double steps[] )
{
    steps[0] = this->alpha_steps_per_mm;
    steps[1] = this->beta_steps_per_mm;
    steps[2] = this->gamma_steps_per_mm;
}

