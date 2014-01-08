#include "RostockSolution.h"
#include <fastmath.h>

#define PIOVER180       0.01745329251994329576923690768489F

RostockSolution::RostockSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();

    float alpha_angle  = PIOVER180 * this->config->value(alpha_angle_checksum)->by_default(30.0f)->as_number();
    this->sin_alpha     = sinf(alpha_angle);
    this->cos_alpha     = cosf(alpha_angle);
    float beta_angle   = PIOVER180 * this->config->value( beta_relative_angle_checksum)->by_default(120.0f)->as_number();
    this->sin_beta      = sinf(beta_angle);
    this->cos_beta      = cosf(beta_angle);
    float gamma_angle  = PIOVER180 * this->config->value(gamma_relative_angle_checksum)->by_default(240.0f)->as_number();
    this->sin_gamma     = sinf(gamma_angle);
    this->cos_gamma     = cosf(gamma_angle);

    // arm_length is the length of the arm from hinge to hinge
    this->arm_length         = this->config->value(arm_length_checksum)->by_default(250.0f)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    this->arm_radius         = this->config->value(arm_radius_checksum)->by_default(124.0f)->as_number();

    this->arm_length_squared = powf(this->arm_length, 2);
}

void RostockSolution::millimeters_to_steps( float millimeters[], int steps[] ){
    float mm[3], alpha_rotated[3], rotated[3];
    
    // convert input to float
    mm[0]= millimeters[0];
    mm[1]= millimeters[1];
    mm[2]= millimeters[2];
    
    if( this->sin_alpha == 0 && this->cos_alpha == 1){
        alpha_rotated[0] = mm[0];
        alpha_rotated[1] = mm[1];
        alpha_rotated[2] = mm[2];
    }else{
        rotate( mm, alpha_rotated, sin_alpha, cos_alpha );
    }
    steps[ALPHA_STEPPER] = lround( solve_arm( alpha_rotated ) * this->alpha_steps_per_mm );

    rotate( alpha_rotated, rotated, sin_beta, cos_beta );
    steps[BETA_STEPPER ] = lround( solve_arm( rotated ) * this->beta_steps_per_mm );

    rotate( alpha_rotated, rotated, sin_gamma, cos_gamma );
    steps[GAMMA_STEPPER] = lround( solve_arm( rotated ) * this->gamma_steps_per_mm );
}

void RostockSolution::steps_to_millimeters( int steps[], float millimeters[] ){} 

float RostockSolution::solve_arm( float millimeters[]) {
    return sqrtf(arm_length_squared - powf(millimeters[X_AXIS] - this->arm_radius, 2) - powf(millimeters[Y_AXIS], 2)) + millimeters[Z_AXIS];
}

void RostockSolution::rotate(float in[], float out[], float sin, float cos ){
    out[X_AXIS] = cos * in[X_AXIS] - sin * in[Y_AXIS];
    out[Y_AXIS] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    out[Z_AXIS] = in[Z_AXIS];
}

void RostockSolution::set_steps_per_millimeter( float steps[] )
{
    this->alpha_steps_per_mm = steps[0];
    this->beta_steps_per_mm  = steps[1];
    this->gamma_steps_per_mm = steps[2];
}

void RostockSolution::get_steps_per_millimeter( float steps[] )
{
    steps[0] = this->alpha_steps_per_mm;
    steps[1] = this->beta_steps_per_mm;
    steps[2] = this->gamma_steps_per_mm;
}

