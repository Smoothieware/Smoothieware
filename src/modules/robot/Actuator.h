#ifndef _ACTUATOR_H
#define _ACTUATOR_H

#include <cstddef>
#include <math.h>

class Endstop;

class Actuator
{
public:
    Actuator()
    {
        name_checksum  = 0;
        max_speed      = 100.0F;
        accel          = 100.0F;
        soft_min       = NAN;
        soft_max       = NAN;
        last_milestone = 0.0F;
    };

    uint16_t name_checksum;

    float max_speed;
    float accel;

    float soft_min;
    float soft_max;

    float last_milestone;
};

#endif /* _ACTUATOR_H */
