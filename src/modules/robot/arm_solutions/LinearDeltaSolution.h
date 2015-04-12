#ifndef LINEARDELTASOLUTION_H
#define LINEARDELTASOLUTION_H
#include "libs/Module.h"
#include "BaseSolution.h"

// You have to include this to get memcpy
#include <cstring>

#define X 0
#define Y 1
#define Z 2

class Config;

class LinearDeltaSolution : public BaseSolution {

    public:
        LinearDeltaSolution(Config*);
        void cartesian_to_actuator( float[], float[] );
        void actuator_to_cartesian( float[], float[] );
        void get_tower_xyz_for_dist(uint8_t tower, float xyz[], float dist);

        bool set_optional(const arm_options_t& options);
        bool get_optional(arm_options_t& options);

    private:
        void init();

        float arm_length;
        float arm_radius;		// Same as "delta radius" in other delta robot control software
        float arm_length_squared;

        // { X, Y } location of each tower's top
        float delta_tower1_x;
        float delta_tower1_y;
        float delta_tower2_x;
        float delta_tower2_y;
        float delta_tower3_x;
        float delta_tower3_y;

        // Distance-from-center offset for each tower
        float tower1_offset;
        float tower2_offset;
        float tower3_offset;

        // Offset from ideal angle (e.g.: Z tower is 90 degrees + tower3_angle)
        float tower1_angle;
        float tower2_angle;
        float tower3_angle;

        // Tower lean: Unit vector pointing from top of tower, to bottom of tower :~)
        float tower1_vector[3];
        float tower2_vector[3];
        float tower3_vector[3];


};
#endif // LINEARDELTASOLUTION_H
