#include "CartesianSolution.h"
#include <math.h>

CartesianSolution::CartesianSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->get(alpha_steps_per_mm_checksum);
    this->beta_steps_per_mm  = this->config->get( beta_steps_per_mm_checksum);
    this->gamma_steps_per_mm = this->config->get(gamma_steps_per_mm_checksum);
}

void CartesianSolution::millimeters_to_steps( double millimeters[], int steps[] ){
    steps[ALPHA_STEPPER] = lround( millimeters[X_AXIS] * this->alpha_steps_per_mm );
    steps[BETA_STEPPER ] = lround( millimeters[Y_AXIS] * this->beta_steps_per_mm );
    steps[GAMMA_STEPPER] = lround( millimeters[Z_AXIS] * this->gamma_steps_per_mm );
}

void CartesianSolution::steps_to_millimeters( int steps[], double millimeters[] ){} 
