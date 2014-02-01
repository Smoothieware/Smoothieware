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
};

#endif /* _CARTESIANAXIS_H */
