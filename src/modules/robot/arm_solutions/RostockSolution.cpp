#include "RostockSolution.h"
#include <math.h>

RostockSolution::RostockSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();
    this->arm_length         = this->config->value(rostock_arm_length_checksum)->as_number();
    this->arm_radius         = this->config->value(rostock_arm_radius_checksum)->as_number();
    this->carriage_radius    = this->config->value(rostock_carriage_radius_checksum)->as_number();
}

void RostockSolution::millimeters_to_steps( double millimeters[], int steps[] ){
    double temp_millis[3];
    steps[ALPHA_STEPPER] = lround( solve_arm( millimeters ) * this->alpha_steps_per_mm );

    rotate_120(millimeters, temp_millis);
    steps[BETA_STEPPER ] = lround( solve_arm( temp_millis ) * this->beta_steps_per_mm );

    rotate_240(millimeters, temp_millis);
    steps[GAMMA_STEPPER] = lround( solve_arm( temp_millis ) * this->gamma_steps_per_mm );
}

void RostockSolution::steps_to_millimeters( int steps[], double millimeters[] ){} 

double RostockSolution::solve_arm( double millimeters[]) {
    double arm_xlift = sqrt( pow(this->arm_length,2) - pow( ((this->arm_radius - this->carriage_radius) - millimeters[X_AXIS]),2 ) );
    double arm_ylift = sqrt( pow(arm_xlift,2) - pow( - millimeters[Y_AXIS],2 ) ) + arm_xlift;
    return arm_xlift - arm_ylift + millimeters[Z_AXIS];
}

#define SIN60 0.8660254037844386
#define COS60 0.5
void RostockSolution::rotate_120( double coords[], double out[] ){
    out[X_AXIS] = - COS60 * coords[X_AXIS] - SIN60 * coords[Y_AXIS];
    out[Y_AXIS] = SIN60 * coords[X_AXIS] - COS60 * coords[Y_AXIS];
    out[Z_AXIS] = coords[Z_AXIS];
}

void RostockSolution::rotate_240( double coords[], double out[] ){
    out[X_AXIS] = - COS60 * coords[X_AXIS] + SIN60 * coords[Y_AXIS];
    out[Y_AXIS] = - SIN60 * coords[X_AXIS] - COS60 * coords[Y_AXIS];
    out[Z_AXIS] = coords[Z_AXIS];
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

