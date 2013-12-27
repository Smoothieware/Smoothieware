#include "JohannKosselSolution.h"
#include <fastmath.h>

#define PIOVER180       0.01745329251994329576923690768489F
#define SQ(x) powf(x, 2)

JohannKosselSolution::JohannKosselSolution(Config* passed_config) : config(passed_config){
    this->alpha_steps_per_mm = this->config->value(alpha_steps_per_mm_checksum)->as_number();
    this->beta_steps_per_mm  = this->config->value( beta_steps_per_mm_checksum)->as_number();
    this->gamma_steps_per_mm = this->config->value(gamma_steps_per_mm_checksum)->as_number();

    // arm_length is the length of the arm from hinge to hinge
    this->arm_length         = this->config->value(arm_length_checksum)->by_default(250.0f)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    this->arm_radius         = this->config->value(arm_radius_checksum)->by_default(124.0f)->as_number();

    init();
}

void JohannKosselSolution::init() {
    this->arm_length_squared = SQ(this->arm_length);

    // Effective X/Y positions of the three vertical towers.
    float DELTA_RADIUS= this->arm_radius;
    float SIN_60= 0.8660254037844386F;
    float COS_60= 0.5F;
    DELTA_TOWER1_X= -SIN_60*DELTA_RADIUS; // front left tower
    DELTA_TOWER1_Y= -COS_60*DELTA_RADIUS;
    DELTA_TOWER2_X= SIN_60*DELTA_RADIUS; // front right tower
    DELTA_TOWER2_Y= -COS_60*DELTA_RADIUS;
    DELTA_TOWER3_X= 0.0F; // back middle tower
    DELTA_TOWER3_Y= DELTA_RADIUS;
}

void JohannKosselSolution::millimeters_to_steps( float millimeters[], int steps[] ){

    float delta_x = sqrtf(this->arm_length_squared
                         - SQ(DELTA_TOWER1_X-millimeters[0])
                         - SQ(DELTA_TOWER1_Y-millimeters[1])
                        ) + millimeters[2];
    float delta_y = sqrtf(this->arm_length_squared
                         - SQ(DELTA_TOWER2_X-millimeters[0])
                         - SQ(DELTA_TOWER2_Y-millimeters[1])
                        ) + millimeters[2];
    float delta_z = sqrtf(this->arm_length_squared
                         - SQ(DELTA_TOWER3_X-millimeters[0])
                         - SQ(DELTA_TOWER3_Y-millimeters[1])
                        ) + millimeters[2];


    steps[ALPHA_STEPPER] = lround( delta_x * this->alpha_steps_per_mm );
    steps[BETA_STEPPER ] = lround( delta_y * this->beta_steps_per_mm );
    steps[GAMMA_STEPPER] = lround( delta_z * this->gamma_steps_per_mm );
}

void JohannKosselSolution::steps_to_millimeters( int steps[], float millimeters[] ){}

void JohannKosselSolution::set_steps_per_millimeter( float steps[] )
{
    this->alpha_steps_per_mm = steps[0];
    this->beta_steps_per_mm  = steps[1];
    this->gamma_steps_per_mm = steps[2];
}

void JohannKosselSolution::get_steps_per_millimeter( float steps[] )
{
    steps[0] = this->alpha_steps_per_mm;
    steps[1] = this->beta_steps_per_mm;
    steps[2] = this->gamma_steps_per_mm;
}

bool JohannKosselSolution::set_optional(char parameter, float value) {

    switch(parameter) {
        case 'L': // sets arm_length
            this->arm_length= value;
            init();
            break;
        case 'R': // sets arm_radius
            this->arm_radius= value;
            init();
            break;
        default:
            return false;
    }
    return true;
}

bool JohannKosselSolution::get_optional(char parameter, float *value) {
    if(value == NULL) return false;

    switch(parameter) {
        case 'L': // get arm_length
            *value= this->arm_length;
            break;
        case 'R': // get arm_radius
            *value= this->arm_radius;
            break;
        default:
            return false;
    }

    return true;
};
