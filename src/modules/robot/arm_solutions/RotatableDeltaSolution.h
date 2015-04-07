#ifndef RotatableDeltaSolution_H
#define RotatableDeltaSolution_H
#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class RotatableDeltaSolution : public BaseSolution {
    public:
        RotatableDeltaSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );

        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);

    private:
        void init();
        int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
        int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);

        float delta_e;			// End effector length
        float delta_f;			// Base length
        float delta_re;			// Carbon rod length
        float delta_rf;			// Servo horn length
        float delta_z_offset ;	// Distance from delta 8mm rod/pulley to table/bed. <- LERCHE 25-09-2014 uncommented

        //NOTE: For OpenPnP, set the zero to be about 25mm above the bed...
        //float delta_z_offset;	// Distance from delta 8mm rod/pulley to table/bed. <- LERCHE 25-09-2014 commented

        float delta_ee_offs;	// Ball joint plane to bottom of end effector surface
        //float tool_offset;	// No offset
        //float tool_offset;	// Distance between end effector ball joint plane and tip of tool (Z probe)
        float tool_offset;		// Distance between end effector ball joint plane and tip of tool (PnP)
        float z_calc_offset;

        float z_home_angle;		// This is the angle where the arms hit the endstop sensor
        float z_home_offs;		// This is calculated from the above angle, after applying forward
        						// kinematics, and adding the Z calc offset to it.

        // Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
        float delta_printable_radius;

        float xyz_full_steps_per_rotation;	// stepper motor steps per 360 full rotation
											// for 1.8 degree this is 200 for 0.9 degree this is 400
        float xyz_microsteps;				//stepper motor drive usually 8.0, 16.0 or 32.0
        float small_pulley_teeth;			// how many teeth as on the pulley on the stepper motor
        float big_pulley_teeth;				//how many teeth are on the main pulley note if your robot does not use a pulley
											// reduction system, make it the same as small_pulley_teeth to get 1:1

        float delta[3] = {0.0, 0.0, 0.0};
};
#endif // RotatableDeltaSolution_H
