// Base class for an arm solution, only usefull for inheritence. http://en.wikipedia.org/wiki/Arm_solution
#ifndef BASESOLUTION_H
#define BASESOLUTION_H

#include <map>
#include "ActuatorCoordinates.h"

class Config;

class BaseSolution {
    public:
        BaseSolution(){};
        BaseSolution(Config*){};
        virtual ~BaseSolution() {};
        virtual void cartesian_to_actuator(const float[], ActuatorCoordinates &) const = 0;
        virtual void actuator_to_cartesian(const ActuatorCoordinates &, float[]) const = 0;
        typedef std::map<char, float> arm_options_t;
        virtual bool set_optional(const arm_options_t& options) { return false; };
        virtual bool get_optional(arm_options_t& options, bool force_all= false) const { return false; };
};

#endif
