#include "RotatableCartesianSolution.h"
#include <math.h>

#define PIOVER180       0.01745329251994329576923690768489

RotatableCartesianSolution::RotatableCartesianSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();

    float alpha_angle  = PIOVER180 * this->config->value(alpha_angle_checksum)->by_default(0.0f)->as_number();
    this->sin_alpha     = sinf(alpha_angle);
    this->cos_alpha     = cosf(alpha_angle);
}

void RotatableCartesianSolution::millimeters_to_steps( float millimeters[], int steps[] ){
    float rotated[3];
    rotate( millimeters, rotated, this->sin_alpha, this->cos_alpha );

    steps[ALPHA_STEPPER] = lround( rotated[X_AXIS] * this->alpha_steps_per_mm );
    steps[BETA_STEPPER ] = lround( rotated[Y_AXIS] * this->beta_steps_per_mm );
    steps[GAMMA_STEPPER] = lround( rotated[Z_AXIS] * this->gamma_steps_per_mm );
}

void RotatableCartesianSolution::steps_to_millimeters( int steps[], float millimeters[] ){
    float rotated[3];

    rotated[ALPHA_STEPPER] = steps[X_AXIS] / this->alpha_steps_per_mm;
    rotated[BETA_STEPPER ] = steps[Y_AXIS] / this->beta_steps_per_mm;
    rotated[GAMMA_STEPPER] = steps[Z_AXIS] / this->gamma_steps_per_mm;

    rotate( rotated, millimeters, - this->sin_alpha, this->cos_alpha );
}

void RotatableCartesianSolution::rotate(float in[], float out[], float sin, float cos ){
    out[X_AXIS] = cos * in[X_AXIS] - sin * in[Y_AXIS];
    out[Y_AXIS] = sin * in[X_AXIS] + cos * in[Y_AXIS];
    out[Z_AXIS] = in[Z_AXIS];
}

void RotatableCartesianSolution::set_steps_per_millimeter( float steps[] )
{
    this->alpha_steps_per_mm = steps[0];
    this->beta_steps_per_mm  = steps[1];
    this->gamma_steps_per_mm = steps[2];
}

void RotatableCartesianSolution::get_steps_per_millimeter( float steps[] )
{
    steps[0] = this->alpha_steps_per_mm;
    steps[1] = this->beta_steps_per_mm;
    steps[2] = this->gamma_steps_per_mm;
}
