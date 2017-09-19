#pragma once

#include "libs/Module.h"
#include "BaseSolution.h"

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

        float arm_length;
        float arm_radius;
        float arm_length_squared;

        float delta_tower1_x;
        float delta_tower1_y;
        float delta_tower2_x;
        float delta_tower2_y;
        float delta_tower3_x;
        float delta_tower3_y;
        float tower1_offset;
        float tower2_offset;
        float tower3_offset;
        float tower1_angle;
        float tower2_angle;
        float tower3_angle;
};
