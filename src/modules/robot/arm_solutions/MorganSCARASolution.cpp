#include "MorganSCARASolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ActuatorCoordinates.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define arm1_length_checksum          CHECKSUM("arm1_length")
#define arm2_length_checksum          CHECKSUM("arm2_length")
#define morgan_offset_x_checksum      CHECKSUM("morgan_offset_x")
#define morgan_offset_y_checksum      CHECKSUM("morgan_offset_y")
#define morgan_scaling_x_checksum     CHECKSUM("morgan_scaling_x")
#define morgan_scaling_y_checksum     CHECKSUM("morgan_scaling_y")
#define morgan_homing_checksum        CHECKSUM("morgan_homing")
#define morgan_undefined_min_checksum CHECKSUM("morgan_undefined_min")
#define morgan_undefined_max_checksum CHECKSUM("morgan_undefined_max")

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
    morgan_offset_y     = config->value(morgan_offset_y_checksum)->by_default(-60.0f)->as_number();
    // Axis scaling is used in final calibration
    morgan_scaling_x    = config->value(morgan_scaling_x_checksum)->by_default(1.0F)->as_number(); // 1 = 100% : No scaling
    morgan_scaling_y    = config->value(morgan_scaling_y_checksum)->by_default(1.0F)->as_number();
    // morgan_undefined is the ratio at which the SCARA position is undefined.
    // required to prevent the arm moving through singularity points
    // min: head close to tower
    morgan_undefined_min  = config->value(morgan_undefined_min_checksum)->by_default(0.95f)->as_number();
    // max: head on maximum reach
    morgan_undefined_max  = config->value(morgan_undefined_max_checksum)->by_default(0.95f)->as_number();

    init();
}

void MorganSCARASolution::init()
{

}

float MorganSCARASolution::to_degrees(float radians) const
{
    return radians * (180.0F / 3.14159265359f);
}

void MorganSCARASolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{

    float SCARA_pos[2],
          SCARA_C2,
          SCARA_S2,
          SCARA_K1,
          SCARA_K2,
          SCARA_theta,
          SCARA_psi;

    SCARA_pos[X_AXIS] = (cartesian_mm[X_AXIS] - this->morgan_offset_x)  * this->morgan_scaling_x;  //Translate cartesian to tower centric SCARA X Y AND apply scaling factor from this offset.
    SCARA_pos[Y_AXIS] = (cartesian_mm[Y_AXIS]  * this->morgan_scaling_y - this->morgan_offset_y);  // morgan_offset not to be confused with home offset. This makes the SCARA math work.
    // Y has to be scaled before subtracting offset to ensure position on bed.

    if (this->arm1_length == this->arm2_length)
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS]) + SQ(SCARA_pos[Y_AXIS]) - 2.0f * SQ(this->arm1_length)) / (2.0f * SQ(this->arm1_length));
    else
        SCARA_C2 = (SQ(SCARA_pos[X_AXIS]) + SQ(SCARA_pos[Y_AXIS]) - SQ(this->arm1_length) - SQ(this->arm2_length)) / (2.0f * SQ(this->arm1_length));

    // SCARA position is undefined if abs(SCARA_C2) >=1
    // In reality abs(SCARA_C2) >0.95 can be problematic.

    if (SCARA_C2 > this->morgan_undefined_max)
        SCARA_C2 = this->morgan_undefined_max;
    else if (SCARA_C2 < -this->morgan_undefined_min)
        SCARA_C2 = -this->morgan_undefined_min;


    SCARA_S2 = sqrtf(1.0f - SQ(SCARA_C2));

    SCARA_K1 = this->arm1_length + this->arm2_length * SCARA_C2;
    SCARA_K2 = this->arm2_length * SCARA_S2;

    SCARA_theta = (atan2f(SCARA_pos[X_AXIS], SCARA_pos[Y_AXIS]) - atan2f(SCARA_K1, SCARA_K2)) * -1.0f; // Morgan Thomas turns Theta in oposite direction
    SCARA_psi   = atan2f(SCARA_S2, SCARA_C2);


    actuator_mm[ALPHA_STEPPER] = to_degrees(SCARA_theta);             // Multiply by 180/Pi  -  theta is support arm angle
    actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_theta + SCARA_psi); // Morgan kinematics (dual arm)
    //actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_psi);             // real scara
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];                // No inverse kinematics on Z - Position to add bed offset?

}

void MorganSCARASolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    // Perform forward kinematics, and place results in cartesian_mm[]

    float y1, y2,
          actuator_rad[2];

    actuator_rad[X_AXIS] = actuator_mm[X_AXIS] / (180.0F / 3.14159265359f);
    actuator_rad[Y_AXIS] = actuator_mm[Y_AXIS] / (180.0F / 3.14159265359f);

    y1 = sinf(actuator_rad[X_AXIS]) * this->arm1_length;
    y2 = sinf(actuator_rad[Y_AXIS]) * this->arm2_length + y1;

    cartesian_mm[X_AXIS] = (((cosf(actuator_rad[X_AXIS]) * this->arm1_length) + (cosf(actuator_rad[Y_AXIS]) * this->arm2_length)) / this->morgan_scaling_x) + this->morgan_offset_x;
    cartesian_mm[Y_AXIS] = (y2 + this->morgan_offset_y) / this->morgan_scaling_y;
    cartesian_mm[Z_AXIS] = actuator_mm[Z_AXIS];

    cartesian_mm[0] = ROUND(cartesian_mm[0], 7);
    cartesian_mm[1] = ROUND(cartesian_mm[1], 7);
    cartesian_mm[2] = ROUND(cartesian_mm[2], 7);
}

bool MorganSCARASolution::set_optional(const arm_options_t& options)
{

    arm_options_t::const_iterator i;

    i = options.find('T');         // Theta arm1 length
    if(i != options.end()) {
        arm1_length = i->second;

    }
    i = options.find('P');         // Psi arm2 length
    if(i != options.end()) {
        arm2_length = i->second;
    }
    i = options.find('X');         // Home initial position X
    if(i != options.end()) {
        morgan_offset_x = i->second;
    }
    i = options.find('Y');         // Home initial position Y
    if(i != options.end()) {
        morgan_offset_y = i->second;
    }
    i = options.find('A');         // Scaling X_AXIS
    if(i != options.end()) {
        morgan_scaling_x = i->second;
    }
    i = options.find('B');         // Scaling Y_AXIS
    if(i != options.end()) {
        morgan_scaling_y = i->second;
    }
    //i= options.find('C');          // Scaling Z_AXIS
    //if(i != options.end()) {
    //    morgan_scaling_z= i->second;
    //}
    i = options.find('D');         // Undefined min
    if(i != options.end()) {
        this->morgan_undefined_min = i->second;
    }
    i = options.find('E');         // undefined max
    if(i != options.end()) {
        this->morgan_undefined_max = i->second;
    }

    init();
    return true;
}

bool MorganSCARASolution::get_optional(arm_options_t& options, bool force_all) const
{
    options['T'] = this->arm1_length;
    options['P'] = this->arm2_length;
    options['X'] = this->morgan_offset_x;
    options['Y'] = this->morgan_offset_y;
    options['A'] = this->morgan_scaling_x;
    options['B'] = this->morgan_scaling_y;
    // options['C']= this->morgan_scaling_z;
    options['D'] = this->morgan_undefined_min;
    options['E'] = this->morgan_undefined_max;

    return true;
};
