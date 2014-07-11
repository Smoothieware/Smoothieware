#include "MorganSCARASolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
//#include "StreamOutputPool.h"
//#include "Gcode.h"
//#include "SerialMessage.h"
//#include "Conveyor.h"
//#include "Robot.h"
//#include "StepperMotor.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define arm1_length_checksum         CHECKSUM("arm1_length")
#define arm2_length_checksum         CHECKSUM("arm2_length")
#define morgan_offset_x_checksum     CHECKSUM("morgan_offset_x")
#define morgan_offset_y_checksum     CHECKSUM("morgan_offset_y")
#define axis_scaling_x_checksum      CHECKSUM("axis_scaling_x")
#define axis_scaling_y_checksum      CHECKSUM("axis_scaling_y")

#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)

MorganSCARASolution::MorganSCARASolution(Config* config)
{
    // arm1_length is the length of the inner main arm from hinge to hinge
    arm1_length         = config->value(arm1_length_checksum)->by_default(150.0f)->as_number();
    // arm2_length is the length of the inner main arm from hinge to hinge
    arm2_length         = config->value(arm2_length_checksum)->by_default(150.0f)->as_number();
    // morgan_offset_x is the x offset of bed zero position towards the SCARA tower center
    morgan_offset_x     = config->value(morgan_offset_x_checksum)->by_default(100.0f)->as_number();
    // morgan_offset_y is the y offset of bed zero position towards the SCARA tower center
    morgan_offset_y     = config->value(morgan_offset_y_checksum)->by_default(-65.0f)->as_number();

    init();
}

void MorganSCARASolution::init() {

}

float MorganSCARASolution::to_degrees(float radians) {
    return radians*(180.0F/3.14159265359f);
}

void MorganSCARASolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{

    float SCARA_pos[2],
          SCARA_C2,
          SCARA_S2,
          SCARA_K1,
          SCARA_K2,
          SCARA_theta,
          SCARA_psi;
  
    SCARA_pos[X_AXIS] = cartesian_mm[X_AXIS] - this->morgan_offset_x;  //Translate cartesian to tower centric SCARA X Y
    SCARA_pos[Y_AXIS] = cartesian_mm[Y_AXIS] - this->morgan_offset_y;  // morgan_offset not to be confused with home offset. Makes the SCARA math work.
 
    if (this->arm1_length == this->arm2_length)
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS])+SQ(SCARA_pos[Y_AXIS])-2.0f*SQ(this->arm1_length)) / (2.0f * SQ(this->arm1_length));
    else
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS])+SQ(SCARA_pos[Y_AXIS])-SQ(this->arm1_length)-SQ(this->arm2_length)) / (2.0f * SQ(this->arm1_length)); 
     
    SCARA_S2 = sqrtf(1.0f-SQ(SCARA_C2));

    SCARA_K1 = this->arm1_length+this->arm2_length*SCARA_C2;
    SCARA_K2 = this->arm2_length*SCARA_S2;
  
    SCARA_theta = (atan2f(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2f(SCARA_K1, SCARA_K2))*-1.0f;    // Morgan Thomas turns Theta in oposite direction
    SCARA_psi   = atan2f(SCARA_S2,SCARA_C2);
  
  
    actuator_mm[ALPHA_STEPPER] = to_degrees(SCARA_theta);             // Multiply by 180/Pi  -  theta is support arm angle
    actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_theta + SCARA_psi); // Morgan kinematics (dual arm)
    //actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_psi);             // real scara
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];                // No inverse kinematics on Z - Position to add bed offset?

}

void MorganSCARASolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] ) {
    // Perform forward kinematics, and place results in cartesian_mm[]
  
    float y1, y2,
           actuator_rad[2];

    actuator_rad[X_AXIS] = actuator_mm[X_AXIS]/(180.0F/3.14159265359f);
    actuator_rad[Y_AXIS] = actuator_mm[Y_AXIS]/(180.0F/3.14159265359f);

    y1 = sinf(actuator_rad[X_AXIS])*this->arm1_length;
    y2 = sinf(actuator_rad[Y_AXIS])*this->arm2_length + y1;
  
    cartesian_mm[X_AXIS] = cosf(actuator_rad[X_AXIS])*this->arm1_length + cosf(actuator_rad[Y_AXIS])*this->arm2_length + this->morgan_offset_x;
    cartesian_mm[Y_AXIS] = y2 + this->morgan_offset_y;
    cartesian_mm[Z_AXIS] = actuator_mm[Z_AXIS];

    cartesian_mm[0] = ROUND(cartesian_mm[0], 4);
    cartesian_mm[1] = ROUND(cartesian_mm[1], 4);
    cartesian_mm[2] = ROUND(cartesian_mm[2], 4);
}

bool MorganSCARASolution::set_optional(const arm_options_t& options) {

    arm_options_t::const_iterator i;

    i= options.find('T');          // Theta arm1 length
    if(i != options.end()) {
        arm1_length= i->second;

    }
    i= options.find('P');          // Psi arm2 length
    if(i != options.end()) {
        arm2_length= i->second;
    }
    i= options.find('X');          // Home initial position X
    if(i != options.end()) {
        morgan_offset_x= i->second;
    }
    i= options.find('Y');          // Home initial position Y
    if(i != options.end()) {
        morgan_offset_y= i->second;
    }
    
    init();
    return true;
}

bool MorganSCARASolution::get_optional(arm_options_t& options) {
    options['T']= this->arm1_length;
    options['P']= this->arm2_length;
    options['X']= this->morgan_offset_x;
    options['Y']= this->morgan_offset_y;
    return true;
};
