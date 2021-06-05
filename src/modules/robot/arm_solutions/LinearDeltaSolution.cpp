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
    halt_on_error= config->value(delta_halt_on_error_checksum)->by_default(true)->as_bool();

    offsets= nullptr;

    // handle offsets if any are defined
    float offs[N_OFFSETS];
    offs[tower1_angle] = config->value(tower1_angle_checksum)->by_default(0.0f)->as_number();
    offs[tower2_angle] = config->value(tower2_angle_checksum)->by_default(0.0f)->as_number();
    offs[tower3_angle] = config->value(tower3_angle_checksum)->by_default(0.0f)->as_number();
    offs[tower1_offset] = config->value(tower1_offset_checksum)->by_default(0.0f)->as_number();
    offs[tower2_offset] = config->value(tower2_offset_checksum)->by_default(0.0f)->as_number();
    offs[tower3_offset] = config->value(tower3_offset_checksum)->by_default(0.0f)->as_number();
    offs[arm1_trim] = config->value(arm1_trim_checksum)->by_default(0.0f)->as_number();
    offs[arm2_trim] = config->value(arm2_trim_checksum)->by_default(0.0f)->as_number();
    offs[arm3_trim] = config->value(arm3_trim_checksum)->by_default(0.0f)->as_number();

    for (int i = 0; i < N_OFFSETS; ++i) {
        if(offs[i] != 0.0f) {
            // if any offsets are defined create array for them and copy all of them
           offsets= new float[N_OFFSETS];
           memcpy(offsets, offs, sizeof(offs));
           break;
        }
    }

    init();
}

LinearDeltaSolution::~LinearDeltaSolution()
{
    if(offsets != nullptr) {
        delete [] offsets;
    }
}

void LinearDeltaSolution::init()
{
    arm_length_squared = SQ(arm_length);

    // Effective X/Y positions of the three vertical towers.
    float delta_radius = arm_radius;

    using std::placeholders::_1;
    using std::placeholders::_2;

    // we trade off memory space here for speed of execution in the cartesian_to_actuator function
    if(offsets != nullptr) {
        delta_tower1_x = (delta_radius + offsets[tower1_offset]) * cosf((210.0F + offsets[tower1_angle]) * PIOVER180); // front left tower
        delta_tower1_y = (delta_radius + offsets[tower1_offset]) * sinf((210.0F + offsets[tower1_angle]) * PIOVER180);
        delta_tower2_x = (delta_radius + offsets[tower2_offset]) * cosf((330.0F + offsets[tower2_angle]) * PIOVER180); // front right tower
        delta_tower2_y = (delta_radius + offsets[tower2_offset]) * sinf((330.0F + offsets[tower2_angle]) * PIOVER180);
        delta_tower3_x = (delta_radius + offsets[tower3_offset]) * cosf(( 90.0F + offsets[tower3_angle]) * PIOVER180); // back middle tower
        delta_tower3_y = (delta_radius + offsets[tower3_offset]) * sinf(( 90.0F + offsets[tower3_angle]) * PIOVER180);
        ik_fnc= std::bind(&LinearDeltaSolution::cartesian_to_actuator_offs, this, _1, _2);
        fk_fnc= std::bind(&LinearDeltaSolution::actuator_to_cartesian_offs, this, _1, _2);

    }else{
        delta_tower1_x = delta_radius * cosf(210.0F * PIOVER180); // front left tower
        delta_tower1_y = delta_radius * sinf(210.0F * PIOVER180);
        delta_tower2_x = delta_radius * cosf(330.0F * PIOVER180); // front right tower
        delta_tower2_y = delta_radius * sinf(330.0F * PIOVER180);
        delta_tower3_x = delta_radius * cosf( 90.0F * PIOVER180); // back middle tower
        delta_tower3_y = delta_radius * sinf( 90.0F * PIOVER180);
        ik_fnc= std::bind(&LinearDeltaSolution::cartesian_to_actuator_no, this, _1, _2);
        fk_fnc= std::bind(&LinearDeltaSolution::actuator_to_cartesian_no, this, _1, _2);
    }
}

void LinearDeltaSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    ik_fnc(cartesian_mm, actuator_mm);

    if(!halt_on_error) return;

    for (int i = 0; i < 3; ++i) {
        if(isnan(actuator_mm[i])) {
            THEKERNEL->streams->printf("error: LinearDelta illegal move. HALTED\n");
            THEKERNEL->call_event(ON_HALT, nullptr);
            break;
        }
    }
}

void LinearDeltaSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
   fk_fnc(actuator_mm, cartesian_mm);
}

void LinearDeltaSolution::cartesian_to_actuator_no(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[ALPHA_STEPPER] = sqrtf(arm_length_squared
                                       - SQ(delta_tower1_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower1_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];
    actuator_mm[BETA_STEPPER ] = sqrtf(arm_length_squared
                                       - SQ(delta_tower2_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower2_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];
    actuator_mm[GAMMA_STEPPER] = sqrtf(arm_length_squared
                                       - SQ(delta_tower3_x - cartesian_mm[X_AXIS])
                                       - SQ(delta_tower3_y - cartesian_mm[Y_AXIS])
                                      ) + cartesian_mm[Z_AXIS];

}

void LinearDeltaSolution::actuator_to_cartesian_no(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    // from http://en.wikipedia.org/wiki/Circumscribed_circle#Barycentric_coordinates_from_cross-_and_dot-products
    // based on https://github.com/ambrop72/aprinter/blob/2de69a/aprinter/printer/DeltaTransform.h#L81
    Vector3 tower1( delta_tower1_x, delta_tower1_y, actuator_mm[0] );
    Vector3 tower2( delta_tower2_x, delta_tower2_y, actuator_mm[1] );
    Vector3 tower3( delta_tower3_x, delta_tower3_y, actuator_mm[2] );

    Vector3 s12 = tower1.sub(tower2);
    Vector3 s23 = tower2.sub(tower3);
    Vector3 s13 = tower1.sub(tower3);

    Vector3 normal = s12.cross(s23);

    float magsq_s12 = s12.magsq();
    float magsq_s23 = s23.magsq();
    float magsq_s13 = s13.magsq();

    float inv_nmag_sq = 1.0F / normal.magsq();
    float q = 0.5F * inv_nmag_sq;

    float a = q * magsq_s23 * s12.dot(s13);
    float b = q * magsq_s13 * s12.dot(s23) * -1.0F; // negate because we use s12 instead of s21
    float c = q * magsq_s12 * s13.dot(s23);

    Vector3 circumcenter( delta_tower1_x * a + delta_tower2_x * b + delta_tower3_x * c,
                          delta_tower1_y * a + delta_tower2_y * b + delta_tower3_y * c,
                          actuator_mm[0] * a + actuator_mm[1] * b + actuator_mm[2] * c );

    float r_sq = 0.5F * q * magsq_s12 * magsq_s23 * magsq_s13;
    float dist = sqrtf(inv_nmag_sq * (arm_length_squared - r_sq));

    Vector3 cartesian = circumcenter.sub(normal.mul(dist));

    cartesian_mm[0] = ROUND(cartesian[0], 4);
    cartesian_mm[1] = ROUND(cartesian[1], 4);
    cartesian_mm[2] = ROUND(cartesian[2], 4);
}

void LinearDeltaSolution::cartesian_to_actuator_offs(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    float arm_length_1_sq = SQ(arm_length + offsets[arm1_trim]);
    float arm_length_2_sq = SQ(arm_length + offsets[arm2_trim]);
    float arm_length_3_sq = SQ(arm_length + offsets[arm3_trim]);

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
}

void LinearDeltaSolution::actuator_to_cartesian_offs(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    Vector3 tower1( delta_tower1_x, delta_tower1_y, actuator_mm[0] );
    Vector3 tower2( delta_tower2_x, delta_tower2_y, actuator_mm[1] );
    Vector3 tower3( delta_tower3_x, delta_tower3_y, actuator_mm[2] );

    float arm_length_1_sq = SQ(arm_length + offsets[arm1_trim]);
    float arm_length_2_sq = SQ(arm_length + offsets[arm2_trim]);
    float arm_length_3_sq = SQ(arm_length + offsets[arm3_trim]);

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
    bool need_offsets= false;
    float offs[N_OFFSETS]{NAN};
    for(auto &i : options) {
        switch(i.first) {
            case 'L': arm_length = i.second; break;
            case 'R': arm_radius = i.second; break;
            case 'A': offs[tower1_offset] = i.second; need_offsets= true; break;
            case 'B': offs[tower2_offset] = i.second; need_offsets= true; break;
            case 'C': offs[tower3_offset] = i.second; need_offsets= true; break;
            case 'D': offs[tower1_angle] = i.second; need_offsets= true; break;
            case 'E': offs[tower2_angle] = i.second; need_offsets= true; break;
            case 'H': offs[tower3_angle] = i.second; need_offsets= true; break;
            case 'I': offs[arm1_trim] = i.second; need_offsets= true; break;
            case 'J': offs[arm2_trim] = i.second; need_offsets= true; break;
            case 'K': offs[arm3_trim] = i.second; need_offsets= true; break;
        }
    }
    if(need_offsets) {
        if(offsets == nullptr) {
            offsets= new float[N_OFFSETS];
            need_offsets= false;
        }

        for (int i = 0; i < N_OFFSETS; ++i) {
            if(!isnan(offs[i])) {
                offsets[i]= offs[i];
            }else if(!need_offsets) {
                offsets[i]= 0.0f;
            }
        }
    }

    // TODO if we went from offsets to no offsets delete the offsets array

    init();
    return true;
}

bool LinearDeltaSolution::get_optional(arm_options_t& options, bool force_all) const
{
    options['L'] = this->arm_length;
    options['R'] = this->arm_radius;

    // don't report these if none of them are set
    if(offsets != nullptr) {
        options['A'] = offsets[tower1_offset];
        options['B'] = offsets[tower2_offset];
        options['C'] = offsets[tower3_offset];
        options['D'] = offsets[tower1_angle];
        options['E'] = offsets[tower2_angle];
        options['H'] = offsets[tower3_angle];
        options['I'] = offsets[arm1_trim];
        options['J'] = offsets[arm2_trim];
        options['K'] = offsets[arm3_trim];
    }

    return true;
};
