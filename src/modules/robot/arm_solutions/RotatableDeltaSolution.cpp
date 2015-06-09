#include "RotatableDeltaSolution.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "ConfigCache.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"
#include "libs/utils.h"
#include "StreamOutputPool.h"
#include <fastmath.h>

#define delta_e_checksum        				CHECKSUM("delta_e_checksum")
#define delta_f_checksum   		    			CHECKSUM("delta_f_checksum")
#define delta_re_checksum       				CHECKSUM("delta_re_checksum")
#define delta_rf_checksum       				CHECKSUM("delta_rf_checksum")
#define delta_z_offset_checksum      			CHECKSUM("delta_z_offset_checksum")

#define delta_ee_offs_checksum     		  		CHECKSUM("delta_ee_offs_checksum")
#define tool_offset_checksum        			CHECKSUM("tool_offset_checksum")

#define z_home_angle_checksum        			CHECKSUM("z_home_angle_checksum")

#define delta_printable_radius_checksum    		CHECKSUM("delta_printable_radius_checksum")

#define xyz_full_steps_per_rotation_checksum	CHECKSUM("xyz_full_steps_per_rotation_checksum")
#define xyz_microsteps_checksum					CHECKSUM("xyz_microsteps_checksum")
#define small_pulley_teeth_checksum				CHECKSUM("small_pulley_teeth_checksum")
#define big_pulley_teeth_checksum				CHECKSUM("big_pulley_teeth_checksum")

#ifndef alpha_steps_per_mm_checksum
  #define alpha_steps_per_mm_checksum				CHECKSUM("alpha_steps_per_mm_checksum");
#endif

#ifndef beta_steps_per_mm_checksum
  #define beta_steps_per_mm_checksum				CHECKSUM("beta_steps_per_mm_checksum");
#endif

#ifndef gamma_steps_per_mm_checksum
  #define gamma_steps_per_mm_checksum				CHECKSUM("gamma_steps_per_mm_checksum");
#endif

//#define SQ(x) powf(x, 2)
//#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
//#define PIOVER180   0.01745329251994329576923690768489F

//maybe make these #defines like above
  //const float sqrt3 = 1.7320508075688772935274463415059;  // sqrt(3.0)
const float pi     = 3.14159265358979323846;    // PI
const float two_pi = 2 * pi;
const float sin120 = 0.86602540378443864676372317075294; //sqrt3/2.0
const float cos120 = -0.5;
const float tan60  = 1.7320508075688772935274463415059; //sqrt3;
const float sin30  = 0.5;
const float tan30  = 0.57735026918962576450914878050196; //1/sqrt3

RotatableDeltaSolution::RotatableDeltaSolution(Config* config)
{
	// End effector length
	delta_e = config->value((unsigned int)delta_e_checksum)->by_default(131.636f)->as_number();

	// Base length
	delta_f = config->value((unsigned int)delta_f_checksum)->by_default(190.526f)->as_number();

	// Carbon rod length
	delta_re = config->value((unsigned int)delta_re_checksum)->by_default(270.000f)->as_number();

	// Servo horn length
	delta_rf = config->value((unsigned int)delta_rf_checksum)->by_default(90.000f)->as_number();

	// Distance from delta 8mm rod/pulley to table/bed,
	// NOTE: For OpenPnP, set the zero to be about 25mm above the bed..
	delta_z_offset = config->value((unsigned int)delta_z_offset_checksum)->by_default(290.700f)->as_number();

	// Ball joint plane to bottom of end effector surface
	delta_ee_offs = config->value((unsigned int)delta_ee_offs_checksum)->by_default(15.000f)->as_number();

	// Distance between end effector ball joint plane and tip of tool (PnP)
	tool_offset = config->value((unsigned int)tool_offset_checksum)->by_default(30.500f)->as_number();

	// This is the angle where the arms hit the endstop sensor
	z_home_angle = config->value((unsigned int)z_home_angle_checksum)->by_default(-60.000f)->as_number();

	// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
	delta_printable_radius = config->value((unsigned int)delta_printable_radius_checksum)->by_default(150.0f)->as_number();

	//While the by_default() method fills in default values if the field is missing from the config
	//it does not prevent crazy numbers being inputed, so we try to prevent odd ratios and divide by zero
	//errors in our configuration calculations
	xyz_full_steps_per_rotation = config->value((unsigned int)xyz_full_steps_per_rotation_checksum)->by_default(200.0f)->as_number();
	if (xyz_full_steps_per_rotation < 1) xyz_full_steps_per_rotation = 200.0; //try to prevent stupid config entries

	xyz_microsteps = config->value((unsigned int)xyz_microsteps_checksum)->by_default(16.0f)->as_number();
	if (xyz_microsteps < 1) xyz_microsteps = 16.0; //try to prevent stupid config entries

	small_pulley_teeth = config->value((unsigned int)small_pulley_teeth_checksum)->by_default(16.0f)->as_number();
	if (small_pulley_teeth < 1) small_pulley_teeth = 16.0; //try to prevent stupid config entries

	big_pulley_teeth = config->value((unsigned int)big_pulley_teeth_checksum)->by_default(150.0f)->as_number();
	if (big_pulley_teeth < 1) big_pulley_teeth = 150.0; //try to prevent stupid config entries

	//
	//We need to override the alpha_steps_per_mm, beta_steps_per_mm & gamma_steps_per_mm read from the config file
	//as these must be calculated based on other parameters in the config file on the SD card.
	//The formula is:
	//     (xyz_full_steps_per_rotation*xyz_microsteps*(big_pulley_teeth/small_pulley_teeth))*(two_pi*delta_rf)
	//
	//which means:
	//
	//xyz_full_steps_per_rotation*xyz_microsteps gives the number of steps the stepper motor needs to rotate 360 degrees
	//
	//(big_pulley_teeth/small_pulley_teeth) gives the ratio of the large pulley to the small one so that we can determine
	//the  number of rotations the stepper motor needs to rotate the large pulley fully 360 degrees
	//
	//and the two_pi*delta_rf gives the circumference of the large pulley itself.
	//
	//So the formula informs us the number of steps the stepper motor needs to move a mm on the large pulley.
	//
	//For example, a 0.9 degree stepper has 400 steps per revolution, driven using a 1/32 microstep DRV8825 driver chip
	//with a 16 tooth small pulley on the stepper motor and a 150 tooth (equivalent) large pulley of radius 90mm results in:
	//
	//(400*32*(150/16))/(2 * pi * 90) = 212.206590789194 microsteps are required to move the effector attached to
	//the large pulley 1mm in a linear sense.
	//
	//Lastly, if you are not using a small and large pulley system like the FirePick is, simply make these two values
	//the same, BUT NOT ZERO.

	init();
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int RotatableDeltaSolution::delta_calcAngleYZ(float x0, float y0, float z0, float &theta)
{
    float y1 = -0.5 * tan30 * delta_f; // f/2 * tan 30
    y0 -= 0.5 * tan30       * delta_e; // shift center to edge
    // z = a + b*y
    float a = (x0*x0 + y0*y0 + z0*z0 + delta_rf*delta_rf - delta_re*delta_re - y1*y1) / (2.0*z0);
    float b = (y1-y0)/z0;

    float d = -(a+b*y1)*(a+b*y1) + delta_rf*(b*b*delta_rf+delta_rf); // discriminant
    if (d < 0) return -1;                                            // non-existing point

    float yj = (y1 - a*b - sqrt(d))/(b*b + 1.0);                     // choosing outer point
    float zj = a + b*yj;

    theta = 180.0F*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
    return 0;
}

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int RotatableDeltaSolution::delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0)
{
     float t = (delta_f-delta_e)*tan30/2.0F;
     float degrees_to_radians = pi/180.0F;

     theta1 *= degrees_to_radians;
     theta2 *= degrees_to_radians;
     theta3 *= degrees_to_radians;

     float y1 = -(t + delta_rf*cos(theta1));
     float z1 = -delta_rf*sin(theta1);

     float y2 = (t + delta_rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -delta_rf*sin(theta2);

     float y3 = (t + delta_rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -delta_rf*sin(theta3);

     float dnm = (y2-y1)*x3-(y3-y1)*x2;

     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;

     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - delta_re*delta_re);

     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point

     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;

     z0 += z_calc_offset; //nj
     return 0;
}


void RotatableDeltaSolution::init() {

	z_calc_offset  = (delta_z_offset - tool_offset - delta_ee_offs)*-1;

    // This is calculated from the Z_HOME_ANGLE angles specified in the config (file), after applying forward
    // kinematics, and adding the Z calc offset to it.
	z_home_offs    = (((delta_z_offset - tool_offset - delta_ee_offs) - 182.002) - 0.5);

}

void RotatableDeltaSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
	//We need to translate the Cartesian coordinates in mm to the actuator position required in mm so the stepper motor  functions
	float alpha_theta = 0.0F;
	float beta_theta  = 0.0F;
	float gamma_theta = 0.0F;

	//Code from Trossen Robotics tutorial.

	float x0 = cartesian_mm[X_AXIS];
	float y0 = cartesian_mm[Y_AXIS];
	float z_with_offset = cartesian_mm[Z_AXIS] + z_calc_offset; //The delta calculation below places zero at the top.  Subtract the Z offset to make zero at the bottom.

	int status =              delta_calcAngleYZ(x0,                    y0,                  z_with_offset, alpha_theta);
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z_with_offset, beta_theta);  // rotate co-ordinates to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z_with_offset, gamma_theta);  // rotate co-ordinates to -120 deg

	if (status == -1)  //something went wrong
	{
	    THEKERNEL->streams->printf("ERROR: Delta calculation fail!  Unable to move to:\n");
	    THEKERNEL->streams->printf("    x= %f\n",cartesian_mm[X_AXIS]);
	    THEKERNEL->streams->printf("    y= %f\n",cartesian_mm[Y_AXIS]);
	    THEKERNEL->streams->printf("    z= %f\n",cartesian_mm[Z_AXIS]);
	    THEKERNEL->streams->printf(" CalcZ= %f\n",z_calc_offset);
	    THEKERNEL->streams->printf(" Offz= %f\n",z_with_offset);
	  }
	  else
	  {
		  //The Trossen tutorial puts the "X" in the front/middle, FPD puts this arm in the back/middle for aesthetics.
		  //so we make them negative!

		  actuator_mm[ALPHA_STEPPER] = alpha_theta;
		  actuator_mm[BETA_STEPPER ] = beta_theta;
		  actuator_mm[GAMMA_STEPPER] = gamma_theta;

//		  THEKERNEL->streams->printf("cartesian x= %f\n\r",cartesian_mm[X_AXIS]);
//		  THEKERNEL->streams->printf(" y= %f\n\r",cartesian_mm[Y_AXIS]);
//		  THEKERNEL->streams->printf(" z= %f\n\r",cartesian_mm[Z_AXIS]);
//		  THEKERNEL->streams->printf(" Offz= %f\n\r",z_with_offset);
//		  THEKERNEL->streams->printf(" delta x= %f\n\r",delta[X_AXIS]);
//		  THEKERNEL->streams->printf(" y= %f\n\r",delta[Y_AXIS]);
//		  THEKERNEL->streams->printf(" z= %f\n\r",delta[Z_AXIS]);
	  }

}

void RotatableDeltaSolution::actuator_to_cartesian( float actuator_mm[], float cartesian_mm[] )
{
    //Use forward kinematics
    delta_calcForward(actuator_mm[ALPHA_STEPPER], actuator_mm[BETA_STEPPER ], actuator_mm[GAMMA_STEPPER], cartesian_mm[X_AXIS], cartesian_mm[Y_AXIS], cartesian_mm[Z_AXIS]);
}

bool RotatableDeltaSolution::set_optional(const arm_options_t& options) {

    for(auto &i : options) {
        switch(i.first) {
            case 'A': delta_e				= i.second; break;
            case 'B': delta_f				= i.second; break;
            case 'C': delta_re			 = i.second; break;
            case 'D': delta_rf				= i.second; break;
            case 'E': delta_z_offset		= i.second; break;
            case 'F': delta_ee_offs		 = i.second; break;
            case 'G': tool_offset			= i.second; break;
            case 'H': z_home_angle		 = i.second; break;
            case 'I': delta_printable_radius    = i.second; break;
        }
    }
    init();
    return true;
}

bool RotatableDeltaSolution::get_optional(arm_options_t& options) {

    // don't report these if none of them are set
    if(this->delta_e     != 0.0F || this->delta_f        != 0.0F || this->delta_re               != 0.0F ||
       this->delta_rf    != 0.0F || this->delta_z_offset != 0.0F || this->delta_ee_offs          != 0.0F ||
       this->tool_offset != 0.0F || this->z_home_angle   != 0.0F || this->delta_printable_radius != 0.0F) {

        options['A'] = this->delta_e;
        options['B'] = this->delta_f;
        options['C'] = this->delta_re;
        options['D'] = this->delta_rf;
        options['E'] = this->delta_z_offset;
        options['F'] = this->delta_ee_offs;
        options['G'] = this->tool_offset;
        options['H'] = this->z_home_angle;
        options['I'] = this->delta_printable_radius;
    }

    return true;
};
