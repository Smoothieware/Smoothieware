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

#define delta_e_checksum       			CHECKSUM("delta_e_checksum")
#define delta_f_checksum   	    		CHECKSUM("delta_f_checksum")
#define delta_re_checksum       		CHECKSUM("delta_re_checksum")
#define delta_rf_checksum       		CHECKSUM("delta_rf_checksum")
#define delta_z_offset_checksum 		CHECKSUM("delta_z_offset_checksum")

#define delta_ee_offs_checksum 		  	CHECKSUM("delta_ee_offs_checksum")
#define tool_offset_checksum			CHECKSUM("tool_offset_checksum")

const static float pi     = 3.14159265358979323846;    // PI
const static float two_pi = 2 * pi;
const static float sin120 = 0.86602540378443864676372317075294; //sqrt3/2.0
const static float cos120 = -0.5;
const static float tan60  = 1.7320508075688772935274463415059; //sqrt3;
const static float sin30  = 0.5;
const static float tan30  = 0.57735026918962576450914878050196; //1/sqrt3

RotatableDeltaSolution::RotatableDeltaSolution(Config* config)
{
	// End effector length
	delta_e = config->value(delta_e_checksum)->by_default(131.636F)->as_number();

	// Base length
	delta_f = config->value(delta_f_checksum)->by_default(190.526F)->as_number();

	// Carbon rod length
	delta_re = config->value(delta_re_checksum)->by_default(270.000F)->as_number();

	// Servo horn length
	delta_rf = config->value(delta_rf_checksum)->by_default(90.000F)->as_number();

	// Distance from delta 8mm rod/pulley to table/bed,
	// NOTE: For OpenPnP, set the zero to be about 25mm above the bed..
	delta_z_offset = config->value(delta_z_offset_checksum)->by_default(290.700F)->as_number();

	// Ball joint plane to bottom of end effector surface
	delta_ee_offs = config->value(delta_ee_offs_checksum)->by_default(15.000F)->as_number();

	// Distance between end effector ball joint plane and tip of tool (PnP)
	tool_offset = config->value(tool_offset_checksum)->by_default(30.500F)->as_number();

	init();
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int RotatableDeltaSolution::delta_calcAngleYZ(float x0, float y0, float z0, float &theta)
{
    float y1 = -0.5F * tan30 * delta_f; // f/2 * tan 30
    y0      -=  0.5F * tan30 * delta_e; // shift center to edge
    // z = a + b*y
    float a = (x0*x0 + y0*y0 + z0*z0 + delta_rf*delta_rf - delta_re*delta_re - y1*y1) / (2.0F*z0);
    float b = (y1-y0)/z0;

    float d = -(a+b*y1)*(a+b*y1) + delta_rf*(b*b*delta_rf+delta_rf); // discriminant
    if (d < 0.0F) return -1;                                            // non-existing point

    float yj = (y1 - a*b - sqrtf(d))/(b*b + 1.0F);                     // choosing outer point
    float zj = a + b*yj;

    theta = 180.0F*atanf(-zj/(y1 - yj))/pi + ((yj>y1)?180.0F:0.0F);
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

     float y1 = -(t + delta_rf*cosf(theta1));
     float z1 = -delta_rf*sinf(theta1);

     float y2 = (t + delta_rf*cosf(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -delta_rf*sinf(theta2);

     float y3 = (t + delta_rf*cosf(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -delta_rf*sinf(theta3);

     float dnm = (y2-y1)*x3-(y3-y1)*x2;

     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;

     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0F;

     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0F;

     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2.0F*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - delta_re*delta_re);

     // discriminant
     float d = b*b - (float)4.0F*a*c;
     if (d < 0.0F) return -1; // non-existing point

     z0 = -(float)0.5F*(b+sqrtf(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;

     z0 += z_calc_offset; //nj
     return 0;
}


void RotatableDeltaSolution::init() {

	//these are calculated here and not in the config() as these variables can be fine tuned by the user.
	z_calc_offset  = (delta_z_offset - tool_offset - delta_ee_offs)*-1.0F;
}

void RotatableDeltaSolution::cartesian_to_actuator( float cartesian_mm[], float actuator_mm[] )
{
	//We need to translate the Cartesian coordinates in mm to the actuator position required in mm so the stepper motor  functions
	float alpha_theta = 0.0F;
	float beta_theta  = 0.0F;
	float gamma_theta = 0.0F;

	//Code from Trossen Robotics tutorial, note we put the X axis at the back and not the front of the robot.

	float x0 = cartesian_mm[X_AXIS];
	float y0 = cartesian_mm[Y_AXIS];
	float z_with_offset = cartesian_mm[Z_AXIS] + z_calc_offset; //The delta calculation below places zero at the top.  Subtract the Z offset to make zero at the bottom.

	int status =              delta_calcAngleYZ(x0,                    y0,                  z_with_offset, alpha_theta);
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z_with_offset, beta_theta);  // rotate co-ordinates to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z_with_offset, gamma_theta);  // rotate co-ordinates to -120 deg

	if (status == -1)  //something went wrong,
	{
	    //force to actuator FPD home position as we know this is a valid position
	    actuator_mm[ALPHA_STEPPER] = 0;
	    actuator_mm[BETA_STEPPER ] = 0;
	    actuator_mm[GAMMA_STEPPER] = 0;

//DEBUG CODE, uncomment the following to help determine what may be happening if you are trying to adapt this to your own different roational delta.
//	    THEKERNEL->streams->printf("ERROR: Delta calculation fail!  Unable to move to:\n");
//	    THEKERNEL->streams->printf("    x= %f\n",cartesian_mm[X_AXIS]);
//	    THEKERNEL->streams->printf("    y= %f\n",cartesian_mm[Y_AXIS]);
//	    THEKERNEL->streams->printf("    z= %f\n",cartesian_mm[Z_AXIS]);
//	    THEKERNEL->streams->printf(" CalcZ= %f\n",z_calc_offset);
//	    THEKERNEL->streams->printf(" Offz= %f\n",z_with_offset);
	  }
	  else
	  {
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
            case 'A': delta_e			= i.second; break;
            case 'B': delta_f			= i.second; break;
            case 'C': delta_re			= i.second; break;
            case 'D': delta_rf			= i.second; break;
            case 'E': delta_z_offset		= i.second; break;
            case 'F': delta_ee_offs		= i.second; break;
            case 'H': tool_offset		= i.second; break;
        }
    }
    init();
    return true;
}

bool RotatableDeltaSolution::get_optional(arm_options_t& options) {

    // don't report these if none of them are set
    if(this->delta_e     != 0.0F || this->delta_f        != 0.0F || this->delta_re               != 0.0F ||
       this->delta_rf    != 0.0F || this->delta_z_offset != 0.0F || this->delta_ee_offs          != 0.0F ||
       this->tool_offset != 0.0F) {

        options['A'] = this->delta_e;
        options['B'] = this->delta_f;
        options['C'] = this->delta_re;
        options['D'] = this->delta_rf;
        options['E'] = this->delta_z_offset;
        options['F'] = this->delta_ee_offs;
        options['H'] = this->tool_offset;
    }

    return true;
};

