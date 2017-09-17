#pragma once

#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class RotaryDeltaSolution : public BaseSolution {
    public:
        RotaryDeltaSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

        bool set_optional(const arm_options_t& options) override;
        bool get_optional(arm_options_t& options, bool force_all) const override;

    private:
        void init();
        int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) const;
        int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) const;

        float delta_e;			// End effector length
        float delta_f;			// Base length
        float delta_re;			// Carbon rod length
        float delta_rf;			// Servo horn length
        float delta_z_offset ;		// Distance from delta 8mm rod/pulley to table/bed
			       		// NOTE: For OpenPnP, set the zero to be about 25mm above the bed

        float delta_ee_offs;		// Ball joint plane to bottom of end effector surface
        float tool_offset;		// Distance between end effector ball joint plane and tip of tool
        float z_calc_offset;

        struct {
            bool debug_flag:1;
            bool mirror_xy:1;
        };
};
