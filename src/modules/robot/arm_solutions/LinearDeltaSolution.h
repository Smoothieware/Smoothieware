#pragma once

#include "libs/Module.h"
#include "BaseSolution.h"

#include <functional>

class Config;

class LinearDeltaSolution : public BaseSolution {
    public:
        LinearDeltaSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

        bool set_optional(const arm_options_t& options) override;
        bool get_optional(arm_options_t& options, bool force_all) const override;

    private:
        void init();

        bool halt_on_error;
        float arm_length;
        float arm_radius;
        float arm_length_squared;
        float delta_tower1_x;
        float delta_tower1_y;
        float delta_tower2_x;
        float delta_tower2_y;
        float delta_tower3_x;
        float delta_tower3_y;

        void cartesian_to_actuator_no(const float[], ActuatorCoordinates &) const;
        void actuator_to_cartesian_no(const ActuatorCoordinates &, float[] ) const;
        void cartesian_to_actuator_offs(const float[], ActuatorCoordinates &) const;
        void actuator_to_cartesian_offs(const ActuatorCoordinates &, float[] ) const;
        std::function<void (const float[], ActuatorCoordinates &)> ik_fnc;
        std::function<void (const ActuatorCoordinates &, float[])> fk_fnc;

        float *offsets;
        enum OFFSETS {
            tower1_offset,
            tower2_offset,
            tower3_offset,
            tower1_angle,
            tower2_angle,
            tower3_angle,
            arm1_trim,
            arm2_trim,
            arm3_trim,
            N_OFFSETS
        };
};
