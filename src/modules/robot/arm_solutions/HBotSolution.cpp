#include "HBotSolution.h"
#include <math.h>

HBotSolution::HBotSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();
}

void HBotSolution::millimeters_to_steps( float millimeters[], int steps[] ){
    float delta_x = millimeters[X_AXIS] * this->alpha_steps_per_mm;
    float delta_y = millimeters[Y_AXIS] * this->beta_steps_per_mm;
    steps[ALPHA_STEPPER] = lround(delta_x + delta_y);
    steps[BETA_STEPPER ] = lround(delta_x - delta_y);
    steps[GAMMA_STEPPER] = lround( millimeters[Z_AXIS] * this->gamma_steps_per_mm );
}

void HBotSolution::steps_to_millimeters( int steps[], float millimeters[] ){
    float delta_alpha = steps[X_AXIS] / this->alpha_steps_per_mm;
    float delta_beta = steps[Y_AXIS] / this->beta_steps_per_mm;
    millimeters[ALPHA_STEPPER] = 0.5*(delta_alpha + delta_beta);
    millimeters[BETA_STEPPER ] = 0.5*(delta_alpha - delta_beta);
    millimeters[GAMMA_STEPPER] = steps[Z_AXIS] / this->gamma_steps_per_mm;
}

void HBotSolution::set_steps_per_millimeter( float steps[] )
{
    this->alpha_steps_per_mm = steps[0];
    this->beta_steps_per_mm  = steps[1];
    this->gamma_steps_per_mm = steps[2];
}

void HBotSolution::get_steps_per_millimeter( float steps[] )
{
    steps[0] = this->alpha_steps_per_mm;
    steps[1] = this->beta_steps_per_mm;
    steps[2] = this->gamma_steps_per_mm;
}
