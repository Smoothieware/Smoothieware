#ifndef _CARTESIANAXIS_H
#define _CARTESIANAXIS_H

#include "Actuator.h"

class CartesianAxis : public Actuator
{
public:
    CartesianAxis()
    {
        named_in_gcode = false;
    };

    bool named_in_gcode;

    void change_last_milestone(float milestone)
    {
        last_milestone = milestone;
    };
};

#endif /* _CARTESIANAXIS_H */
