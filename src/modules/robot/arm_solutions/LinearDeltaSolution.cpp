#include "LinearDeltaSolution.h"
#include "ActuatorCoordinates.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"
#include "StreamOutputPool.h"

#include <fastmath.h>
#include "Vector3.h"


#define arm_length_checksum         CHECKSUM("arm_length")
#define arm_radius_checksum         CHECKSUM("arm_radius")

#define tower1_offset_checksum      CHECKSUM("delta_tower1_offset")
#define tower2_offset_checksum      CHECKSUM("delta_tower2_offset")
#define tower3_offset_checksum      CHECKSUM("delta_tower3_offset")
#define tower1_angle_checksum       CHECKSUM("delta_tower1_angle")
#define tower2_angle_checksum       CHECKSUM("delta_tower2_angle")
#define tower3_angle_checksum       CHECKSUM("delta_tower3_angle")
#define arm1_trim_checksum          CHECKSUM("arm1_trim")
#define arm2_trim_checksum          CHECKSUM("arm2_trim")
#define arm3_trim_checksum          CHECKSUM("arm3_trim")
#define delta_halt_on_error_checksum    CHECKSUM("delta_halt_on_error")

#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
#define PIOVER180   0.01745329251994329576923690768489F

LinearDeltaSolution::LinearDeltaSolution(Config* config)
{
    // arm_length is the length of the arm from hinge to hinge
    arm_length = config->value(arm_length_checksum)->by_default(250.0f)->as_number();
    // arm_radius is the horizontal distance from hinge to hinge when the effector is centered
    arm_radius = config->value(arm_radius_checksum)->by_default(124.0f)->as_number();

    tower1_angle = config->value(tower1_angle_checksum)->by_default(0.0f)->as_number();
    tower2_angle = config->value(tower2_angle_checksum)->by_default(0.0f)->as_number();
    tower3_angle = config->value(tower3_angle_checksum)->by_default(0.0f)->as_number();
    tower1_offset = config->value(tower1_offset_checksum)->by_default(0.0f)->as_number();
    tower2_offset = config->value(tower2_offset_checksum)->by_default(0.0f)->as_number();
    tower3_offset = config->value(tower3_offset_checksum)->by_default(0.0f)->as_number();
    arm1_trim = config->value(arm1_trim_checksum)->by_default(0.0f)->as_number();
    arm2_trim = config->value(arm2_trim_checksum)->by_default(0.0f)->as_number();
    arm3_trim = config->value(arm3_trim_checksum)->by_default(0.0f)->as_number();
    halt_on_error= config->value(delta_halt_on_error_checksum)->by_default(true)->as_bool();

    init();
}

void LinearDeltaSolution::init()
{
    arm_length_squared = SQ(arm_length);

    // Effective X/Y positions of the three vertical towers.
    float delta_radius = arm_radius;

    delta_tower1_x = (delta_radius + tower1_offset) * cosf((210.0F + tower1_angle) * PIOVER180); // front left tower
    delta_tower1_y = (delta_radius + tower1_offset) * sinf((210.0F + tower1_angle) * PIOVER180);
    delta_tower2_x = (delta_radius + tower2_offset) * cosf((330.0F + tower2_angle) * PIOVER180); // front right tower
    delta_tower2_y = (delta_radius + tower2_offset) * sinf((330.0F + tower2_angle) * PIOVER180);
    delta_tower3_x = (delta_radius + tower3_offset) * cosf((90.0F  + tower3_angle) * PIOVER180); // back middle tower
    delta_tower3_y = (delta_radius + tower3_offset) * sinf((90.0F  + tower3_angle) * PIOVER180);
}

void LinearDeltaSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{

    float arm_length_1_sq = SQ(arm_length + arm1_trim);
    float arm_length_2_sq = SQ(arm_length + arm2_trim);
    float arm_length_3_sq = SQ(arm_length + arm3_trim);

    actuator_mm[ALPHA_STEPPER] = sqrtf(arm_length_1_sq
                                       - SQ(delta_tower1_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower1_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];
    actuator_mm[BETA_STEPPER ] = sqrtf(arm_length_2_sq
                                       - SQ(delta_tower2_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower2_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];
    actuator_mm[GAMMA_STEPPER] = sqrtf(arm_length_3_sq
                                       - SQ(delta_tower3_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower3_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];

    if(!halt_on_error) return;

    for (int i = 0; i < 3; ++i) {
        if(isnan(actuator_mm[i])) {
            THEKERNEL->streams->printf("error: LinearDelta illegal move. HALTED\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
        }
    }
}

void LinearDeltaSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    Vector3 tower1( delta_tower1_x, delta_tower1_y, actuator_mm[0] );
    Vector3 tower2( delta_tower2_x, delta_tower2_y, actuator_mm[1] );
    Vector3 tower3( delta_tower3_x, delta_tower3_y, actuator_mm[2] );

    float arm_length_1_sq = SQ(arm_length + arm1_trim);
    float arm_length_2_sq = SQ(arm_length + arm2_trim);
    float arm_length_3_sq = SQ(arm_length + arm3_trim);

    Vector3 s12 = tower2.sub(tower1);
    Vector3 s13 = tower3.sub(tower1);

    // Tower 2 sits on the x axis of the new coordinate system, so its x coordinate is the magnitude of s12
    float s12_new_x = s12.mag();

    // Calculating unit x vector
    Vector3 new_x_unit = s12.mul(1/s12_new_x);
    
    // Calculating the transformed x coordinate of tower 3
    float s13_new_x = s13.dot(new_x_unit);

    // Tower 3 sits on the X-Y plane of the new system, so we use its position to calculate the unit y vector
    Vector3 s13_x_comp = new_x_unit.mul(s13_new_x);
    Vector3 s13_y_comp = s13.sub(s13_x_comp);

    float s13_new_y = s13_y_comp.mag();
    Vector3 new_y_unit = s13_y_comp.mul(1/s13_new_y);

    // Cross the unit x and y to get the unit z
    Vector3 new_z_unit = new_x_unit.cross(new_y_unit);

    // Now we can calculate the effector coordinates using the adapted version of Bancroft's algorithm from Wikipedia:
    // https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
    float effector_new_x = (arm_length_1_sq - arm_length_2_sq + SQ(s12_new_x)) / (2 * s12_new_x);
    float effector_new_y = (arm_length_1_sq - arm_length_3_sq + SQ(s13_new_x) + SQ(s13_new_y) - 2 * s13_new_x * effector_new_x) / (2 * s13_new_y);
    float effector_new_z = sqrtf(arm_length_1_sq - SQ(effector_new_x) - SQ(effector_new_y));


    // Convert back to the original coordinates. We negate z because the effector is always under the carriages
    Vector3 cartesian = tower1
                            .add(new_x_unit.mul(effector_new_x))
                            .add(new_y_unit.mul(effector_new_y))
                            .add(new_z_unit.mul(-effector_new_z));

    cartesian_mm[0] = ROUND(cartesian[0], 4);
    cartesian_mm[1] = ROUND(cartesian[1], 4);
    cartesian_mm[2] = ROUND(cartesian[2], 4);
}

bool LinearDeltaSolution::set_optional(const arm_options_t& options)
{

    for(auto &i : options) {
        switch(i.first) {
            case 'L': arm_length = i.second; break;
            case 'R': arm_radius = i.second; break;
            case 'A': tower1_offset = i.second; break;
            case 'B': tower2_offset = i.second; break;
            case 'C': tower3_offset = i.second; break;
            case 'D': tower1_angle = i.second; break;
            case 'E': tower2_angle = i.second; break;
            case 'F': tower3_angle = i.second; break; // WARNING this will be deprecated
            case 'H': tower3_angle = i.second; break;
            case 'I': arm1_trim = i.second; break;
            case 'J': arm2_trim = i.second; break;
            case 'K': arm3_trim = i.second; break;
        }
    }
    init();
    return true;
}

bool LinearDeltaSolution::get_optional(arm_options_t& options, bool force_all) const
{
    options['L'] = this->arm_length;
    options['R'] = this->arm_radius;

    // don't report these if none of them are set
    if(force_all || (this->tower1_offset != 0.0F || this->tower2_offset != 0.0F || this->tower3_offset != 0.0F ||
                     this->tower1_angle != 0.0F  || this->tower2_angle != 0.0F  || this->tower3_angle != 0.0F  ||
                     this->arm1_trim != 0.0F     || this->arm2_trim != 0.0F     || this->arm3_trim != 0.0F) ) {

        options['A'] = this->tower1_offset;
        options['B'] = this->tower2_offset;
        options['C'] = this->tower3_offset;
        options['D'] = this->tower1_angle;
        options['E'] = this->tower2_angle;
        options['H'] = this->tower3_angle;
        options['I'] = this->arm1_trim;
        options['J'] = this->arm2_trim;
        options['K'] = this->arm3_trim;
    }

    return true;
};
