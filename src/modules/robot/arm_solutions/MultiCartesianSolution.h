#ifndef MULTICARTESIANSOLUTION_H
#define MULTICARTESIANSOLUTION_H
#include "BaseSolution.h"

class MultiCartesianSolution : public BaseSolution {
    public:
        MultiCartesianSolution(){};
        MultiCartesianSolution(Config*){};
        void cartesian_to_actuator( const float millimeters[], ActuatorCoordinates &steps ) override;
        void actuator_to_cartesian( const ActuatorCoordinates &steps, float millimeters[] ) override;
        size_t get_actuator_count() const override;
};






#endif
